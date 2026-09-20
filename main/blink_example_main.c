#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>
#include <btstack_stdio_esp32.h>
#include <uni.h>

#include "controller_dac.h"
#include "jetson_link.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcpp"
#include "driver/twai.h"
#pragma GCC diagnostic pop
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"   // brake actuator PWM
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "xbox_dac";

#define I2C_PORT I2C_NUM_0
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_FREQ_HZ 50000

#define MCP4725_ADDR_START 0x60
#define MCP4725_ADDR_END 0x67
#define MCP4725_CMD_WRITE_DAC 0x40

#define DAC_SUPPLY_VOLTAGE 3.30f
#define GAMEPAD_TRIGGER_MAX 1023.0f
#define GAMEPAD_STEERING_MAX 512.0f
#define DAC_UPDATE_PERIOD_MS 20
#define DAC_LOG_PERIOD_MS 500

#define STEERING_ADC_CHANNEL ADC_CHANNEL_6  // GPIO34

// ---------------------------------------------------------------------------
// STEERING ANGLE, calibrated in ADC COUNTS.
//
// The old model converted counts -> volts -> a 0..360 deg absolute angle using
// STEERING_SENSOR_MIN_V/MAX_V, then clamped to [0,360]. Three things were wrong
// with it, and together they made half the steering range read a constant:
//   * it assumed a 10k/10k divider that was not wired (D34 saw the encoder
//     output directly, up to 4.10 V, over GPIO34's absolute maximum)
//   * it assumed ADC full scale = 3.3 V; measured on this chip it is ~3.43 V
//   * the [0,360] clamp then swallowed everything above adc 2960, which
//     included straight-ahead at adc ~3240
//
// Counts avoid all of it: no volts, no 360 deg assumption, no clamp. Calibrate
// by sweeping lock to lock and reading `adc=` out of the log line.
//
// MEASURED 2026-08-20 after rewiring the divider, hand sweep lock to lock.
// Predicted span was 1570; measured 1569. Both endpoints landed ~72 counts
// below prediction -- a constant ADC zero-offset error, gain was correct.
#define STEERING_ADC_LOCK_A 806      // one mechanical stop, D34 0.679 V
#define STEERING_ADC_LOCK_B 2375     // the other,            D34 1.990 V

// The one number still uncalibrated. Counts-to-stops is exact; how many
// PHYSICAL degrees 1569 counts spans has never been measured against the
// steering shaft. Safety does not depend on it -- the end stops below work in
// counts -- but any angle reported to Autoware does. Measure with a protractor
// on the shaft and correct this.
#define STEERING_TRAVEL_DEG 211.2f

#define STEERING_ADC_SAMPLES 12      // the ESP32 ADC is noisy; averaging is not optional
#define STEERING_ADC_RAIL_MARGIN 24  // counts from either end that count as a fault

// END STOPS IN ADC COUNTS, not degrees.
//
// This is the fix for the original finding that the limit was measured from
// wherever the wheel happened to be at power-on. Counts are absolute: they do
// not move when the ESP32 reboots, and they do not depend on the X-button zero
// or on STEERING_TRAVEL_DEG being right.
//
// 7.5% of span (118 counts) of margin at each end.
#define STEERING_ADC_MARGIN 118
#define STEERING_ADC_LIMIT_A (STEERING_ADC_LOCK_A + STEERING_ADC_MARGIN)   //  924
#define STEERING_ADC_LIMIT_B (STEERING_ADC_LOCK_B - STEERING_ADC_MARGIN)   // 2257

// TRUE STRAIGHT-AHEAD, in absolute ADC counts. VEH-13, measured 2026-09-08 by
// three independent hand sweeps: 1527, 1532, 1531 -- sd 2.2 counts. That is 8x
// more repeatable than the mechanical stops (sd 8.6 / 7.0), because how hard you
// push into a stop varies but sighting straight-ahead does not.
//
// The reported angle is measured from HERE, not from the boot position. See
// steering_read_relative_angle().
#define STEERING_ADC_ZERO 1530

// Measured: positive duty drives the count DOWN, toward LOCK_A.
#define STEERING_DUTY_POS_LOWERS_COUNT 1
#define STEERING_MAX_DUTY 0.95f

// Pad deflection below this counts as "centred" for the AUTO handoff.
//
// MEASURED 2026-08-20: this pad's stick does NOT rest at 0. It sat at axis=39
// for 130 continuous seconds, which is 7.6% of full scale and maps to duty
// 0.072 -- enough to creep the steering to -20 deg and hold it there. An
// earlier CAN capture shows the same value as a sustained +0.0761 command.
//
// So 30 was too tight: the arbiter would have read this pad as permanently
// deflected and never allowed AUTO. 60 clears the observed offset with margin.
// The offset itself is a separate problem -- the firmware applies stick input
// with no deadband at all, so the pad commands steering while untouched.
#define STEERING_DEADBAND 60

// The pad is only trusted while the Bluetooth link is up. s_controller_connected
// is cleared solely by an explicit BTstack disconnect callback, so a link that
// wedges without one would otherwise latch the last stick values forever -- the
// window below is the backstop for exactly that.
//
// MEASURED 2026-09-08: this pad transmits ONLY ON CHANGE. Held or centred it
// goes silent, so a short report gap does NOT mean the operator has gone away.
// At 100 ms this made AUTO impossible to engage: the MANUAL->AUTO transition
// requires the pad both CONNECTED and CENTRED, and centred means idle means
// silent means "disconnected". The same bug stopped the wheel mid-stroke and
// latched FAULT_PAD_LOST during autonomy.
//
// An out-of-range or powered-off pad still zeroes the steering IMMEDIATELY,
// via the disconnect callback -- that path is unchanged. Only the mistaken
// inference from silence is gone.
#define CONTROLLER_STALE_US 10000000   // 10 s backstop, was 100 ms

// ---------------------------------------------------------------------------
// BRAKE -- linear actuator on an H-bridge, position feedback on a pot.
// Merged from the brake-integration branch 2026-08-21.
// GPIO32/18/19/35 collide with nothing; 35 is input-only, correct for ADC.
#define BRAKE_EN_PIN GPIO_NUM_32
#define BRAKE_RPWM_PIN GPIO_NUM_18
#define BRAKE_LPWM_PIN GPIO_NUM_19
#define BRAKE_ADC_CHANNEL ADC_CHANNEL_7   // GPIO35 = ADC1_CH7

#define BRAKE_ADC_MIN 10
#define BRAKE_ADC_MAX 400        // 0.9 in of the 4.0in physical stroke (full = 1695)
#define BRAKE_DEADBAND 15
#define BRAKE_SPEED_PWM 255
// Measured 2026-09-08 (brake_R20260908-01, 3006 samples): driving at full PWM
// right up to a 15-count deadband made the actuator overshoot and reverse --
// 283 direction reversals while holding full trigger, pot swinging 35..336
// around a target of 263. Full speed while far keeps application fast; a slow
// approach inside NEAR_BAND lets it stop where it was asked to.
#define BRAKE_SPEED_PWM_NEAR 120
#define BRAKE_NEAR_BAND 45
#define BRAKE_FILTER_ALPHA 0.3f

// Plausibility window for the brake pot, same idea as the steering rail check.
// A disconnected pot reads a rail and the bang-bang loop would then drive the
// actuator into a mechanical stop and hold it there.
#define BRAKE_ADC_RAIL_LO 4
#define BRAKE_ADC_RAIL_HI 4090

#define TWAI_TX_GPIO GPIO_NUM_17
#define TWAI_RX_GPIO GPIO_NUM_16

#define SPARK_MAX_ID 1
#define HEARTBEAT_PERIOD_MS 20

#define DEVICE_TYPE_MOTOR_CONTROLLER 2
#define MANUFACTURER_REV 5
#define FRC_HEARTBEAT_ID 0x01011840
#define API_CLASS_SETPOINT 0
#define API_INDEX_DUTY_CYCLE 2

static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_mcp4725 = NULL;
static adc_oneshot_unit_handle_t s_adc = NULL;
static uint8_t s_mcp4725_address = 0;
// The throttle DAC is OPTIONAL. If it is absent the vehicle still steers, brakes,
// reports telemetry and runs its safety loops -- it simply has no throttle authority.
// A missing throttle actuator must disable THROTTLE, not the whole controller.
static bool s_dac_available = false;
static volatile int32_t s_latest_throttle = 0;
static volatile int32_t s_latest_steering = 0;
static volatile int32_t s_latest_brake = 0;      // LT trigger, 0..1023
static float s_smoothed_brake_adc = -1.0f;      // <0 = uninitialised
static bool s_brake_fault = false;
volatile int s_brake_raw = -1;      // published in $T; -1 = not yet read
static volatile bool s_controller_connected = false;
static volatile int64_t s_last_ctl_us = 0;
static float s_steering_zero_offset_deg = NAN;
static bool s_twai_recovering = false;
static uint32_t s_prev_buttons = 0;

// ---- PAD CRUISE SETPOINT -------------------------------------------------
// For mapping throttle voltage to real speed OUTDOORS, where the operator is on
// the kart and cannot reach a keyboard. Y raises the setpoint, A lowers it,
// B kills it instantly.
//
// MEASURED 2026-09-08 with the wheels JACKED:
//   1.21 V  wheel just begins to turn
//   1.24 V  1 mph      1.39 V  3 mph      1.54 V  5 mph
//   1.31 V  2 mph      1.43 V  4 mph      1.58 V  6 mph
//   fit v_mps = 6.364*V - 7.440, R2 0.988, i.e. 0.064 m/s per 0.01 V
// Loaded on the ground these will all be HIGHER -- that is what the outdoor
// run is for.
//
// SAFETY, all of it already present and relied on here:
//   * the kart's own interlock CUTS throttle whenever the brake is applied
//   * losing the pad zeroes throttle via the disconnect callback
//   * B zeroes the setpoint immediately
//   * the trigger still works and always WINS if it asks for more
#define CRUISE_STEP_V   0.01f
#define CRUISE_MAX_V    1.66f      // never above speed_cap_voltage
static float s_cruise_v = 0.0f;

struct uni_platform* get_my_platform(void);

static int map_val(int x, int in_min, int in_max, int out_min, int out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void brake_stop(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

static void brake_extend(int speed)
{
    // Opposite channel is zeroed FIRST -- never both driven at once.
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, speed);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void brake_retract(int speed)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, speed);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

static esp_err_t brake_init(void)
{
    // ORDER MATTERS. The original raised EN first and configured the PWM pins
    // afterwards, so the H-bridge was enabled while RPWM/LPWM were still
    // floating inputs -- the actuator slammed to full extension on power-up,
    // observed 2026-08-21.
    //
    // Here: hold EN low, drive both PWM pins to a defined low with pulldowns,
    // bring the LEDC channels up at duty 0, then enable the bridge.
    //
    // This does NOT cover power-on until app_main() reaches here, when all
    // three pins float and no code runs. Only PULLDOWN RESISTORS on
    // EN/RPWM/LPWM close that window. Still outstanding hardware work.
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BRAKE_EN_PIN) | (1ULL << BRAKE_RPWM_PIN) |
                        (1ULL << BRAKE_LPWM_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(BRAKE_EN_PIN, 0);
    gpio_set_level(BRAKE_RPWM_PIN, 0);
    gpio_set_level(BRAKE_LPWM_PIN, 0);

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&ledc_timer), TAG, "brake LEDC timer failed");

    ledc_channel_config_t ch_r = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0, .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = BRAKE_RPWM_PIN, .duty = 0, .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_r), TAG, "brake RPWM config failed");

    ledc_channel_config_t ch_l = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0, .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = BRAKE_LPWM_PIN, .duty = 0, .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_l), TAG, "brake LPWM config failed");

    // Both channels now configured at duty 0, so the outputs are held low.
    // Safe to enable the bridge.
    brake_stop();
    gpio_set_level(BRAKE_EN_PIN, 1);
    ESP_LOGI(TAG, "brake bridge enabled (PWM held low first)");

    return ESP_OK;
}

// Drop the H-bridge enable and zero both PWMs. Used on any brake fault: the
// original left EN tied high for the life of the program, which throws away
// the one hardware means of shutting the driver down.
static void brake_disable(void)
{
    brake_stop();
    gpio_set_level(BRAKE_EN_PIN, 0);
}

static void log_twai_status(const char* prefix)
{
    twai_status_info_t status = {0};
    if (twai_get_status_info(&status) != ESP_OK) {
        ESP_LOGW(TAG, "%s: twai_get_status_info failed", prefix);
        return;
    }

    ESP_LOGW(TAG,
             "%s: state=%d tx_err=%u rx_err=%u tx_failed=%u rx_missed=%u tx_q=%u",
             prefix,
             status.state,
             (unsigned)status.tx_error_counter,
             (unsigned)status.rx_error_counter,
             (unsigned)status.tx_failed_count,
             (unsigned)status.rx_missed_count,
             (unsigned)status.msgs_to_tx);
}

static bool steering_can_ensure_ready(void)
{
    twai_status_info_t status = {0};
    if (twai_get_status_info(&status) != ESP_OK) {
        ESP_LOGW(TAG, "twai_get_status_info failed");
        return false;
    }

    if (status.state == TWAI_STATE_RUNNING) {
        if (s_twai_recovering) {
            ESP_LOGI(TAG, "CAN recovered and running");
            s_twai_recovering = false;
        }
        return true;
    }

    if (status.state == TWAI_STATE_BUS_OFF && !s_twai_recovering) {
        ESP_LOGW(TAG, "CAN bus-off detected, starting recovery");
        jetson_link_note_can_fault();
        if (twai_initiate_recovery() == ESP_OK) {
            s_twai_recovering = true;
        } else {
            ESP_LOGW(TAG, "twai_initiate_recovery failed");
        }
        return false;
    }

    if (status.state == TWAI_STATE_RECOVERING) {
        s_twai_recovering = true;
        return false;
    }

    return false;
}

static esp_err_t i2c_master_init_pins(int sda, int scl)
{
    if (s_i2c_bus != NULL) {
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
    }

    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    return i2c_new_master_bus(&bus_config, &s_i2c_bus);
}

static esp_err_t i2c_master_init(void)
{
    return i2c_master_init_pins(SDA_PIN, SCL_PIN);
}

static esp_err_t steering_adc_init(void)
{
    const adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&adc_cfg, &s_adc), TAG, "adc_oneshot_new_unit failed");

    const adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, STEERING_ADC_CHANNEL, &chan_cfg), TAG,
                        "adc_oneshot_config_channel steering failed");
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, BRAKE_ADC_CHANNEL, &chan_cfg), TAG,
                        "adc_oneshot_config_channel brake failed");
    return ESP_OK;
}

static uint32_t make_spark_max_can_id(uint8_t device_id, uint8_t api_class, uint8_t api_index)
{
    return ((uint32_t)DEVICE_TYPE_MOTOR_CONTROLLER << 24) | ((uint32_t)MANUFACTURER_REV << 16) |
           ((uint32_t)api_class << 10) | ((uint32_t)api_index << 6) | (device_id & 0x3F);
}

static esp_err_t steering_can_init(void)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_MODE_NORMAL);
    g_config.tx_queue_len = 16;
    g_config.rx_queue_len = 8;
    g_config.alerts_enabled = TWAI_ALERT_NONE;

    const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_RETURN_ON_ERROR(twai_driver_install(&g_config, &t_config, &f_config), TAG, "twai_driver_install failed");
    ESP_RETURN_ON_ERROR(twai_start(), TAG, "twai_start failed");
    return ESP_OK;
}

static esp_err_t steering_can_send_heartbeat(void)
{
    twai_message_t msg = {
        .extd = 1,
        .identifier = FRC_HEARTBEAT_ID,
        .data_length_code = 8,
    };
    memset(msg.data, 0, sizeof(msg.data));
    msg.data[4] = 0x12;

    return twai_transmit(&msg, pdMS_TO_TICKS(10));
}

static esp_err_t steering_can_send_duty(float duty)
{
    if (duty > STEERING_MAX_DUTY) {
        duty = STEERING_MAX_DUTY;
    } else if (duty < -STEERING_MAX_DUTY) {
        duty = -STEERING_MAX_DUTY;
    }

    twai_message_t msg = {
        .extd = 1,
        .identifier = make_spark_max_can_id(SPARK_MAX_ID, API_CLASS_SETPOINT, API_INDEX_DUTY_CYCLE),
        .data_length_code = 8,
    };
    memset(msg.data, 0, sizeof(msg.data));
    memcpy(&msg.data[0], &duty, sizeof(float));
    msg.data[6] = 0x00;

    return twai_transmit(&msg, pdMS_TO_TICKS(10));
}

// Raw ADC exposed for diagnosis. MEASURED 2026-08-20: at straight-ahead the
// derived angle sits at EXACTLY 0.0 for tens of seconds with none of the +/-1.3
// deg jitter it shows elsewhere, which means absolute_angle is being clamped.
// 4.77 V through the 10k/10k divider is 2.385 V on GPIO34, and ADC_ATTEN_DB_12
// is heavily compressed near there -- so the top of the sensor's swing may be
// past what the ADC can resolve. If s_steering_raw pins at 4095 while the wheel
// still moves, that is confirmed and the divider needs re-ranging.
volatile int s_steering_raw = -1;
volatile float s_steering_sensor_v = 0.0f;

static esp_err_t steering_read_relative_angle(float* relative_angle_deg)
{
    int acc = 0;
    for (int i = 0; i < STEERING_ADC_SAMPLES; ++i) {
        int raw_i = 0;
        ESP_RETURN_ON_ERROR(adc_oneshot_read(s_adc, STEERING_ADC_CHANNEL, &raw_i), TAG,
                            "adc_oneshot_read failed");
        acc += raw_i;
    }
    const int raw = acc / STEERING_ADC_SAMPLES;
    s_steering_raw = raw;
    s_steering_sensor_v = ((float)raw / 4095.0f) * 3.43f * 2.0f;  // diagnosis only

    // Sitting on either rail means the signal is outside what the converter can
    // see -- a wiring fault, not a position. Report it as an error so the
    // caller faults to zero duty instead of believing a saturated number.
    if (raw <= STEERING_ADC_RAIL_MARGIN || raw >= 4095 - STEERING_ADC_RAIL_MARGIN) {
        return ESP_ERR_INVALID_STATE;
    }

    const float span = (float)(STEERING_ADC_LOCK_B - STEERING_ADC_LOCK_A);

    // ABSOLUTE, measured from true straight-ahead -- NOT from the boot position.
    //
    // This used to latch s_steering_zero_offset_deg on the first ADC read after
    // power-up, so "0 degrees" meant "wherever the wheel happened to be when the
    // ESP32 booted". That is the same defect already fixed for the END STOPS,
    // which were made absolute ADC counts for exactly this reason -- but the
    // REPORTED ANGLE was left boot-relative, and that is the number Autoware
    // consumes.
    //
    // Demonstrated 2026-09-08: after the angle-loop runaway the wheel sat at the
    // mechanical lock (ADC 790) and still reported -0.67 deg. Commanding 0 deg
    // would have held it at the lock and called it centre. It also means any
    // steering calibration was silently invalidated by a reboot.
    //
    // STEERING_ADC_ZERO is VEH-13: three hand sweeps on 2026-09-08 gave 1527,
    // 1532, 1531 -- sd 2.2 counts, 8x more repeatable than the mechanical stops.
    //
    // NOTE: STEERING_TRAVEL_DEG (211.2) is still A GUESS; measured travel is
    // ~156 deg. So the SCALE remains wrong until the arc calibration outdoors.
    // The ZERO is now correct, which is what stops the loop parking at a lock.
    *relative_angle_deg = (float)(raw - STEERING_ADC_ZERO) / span * STEERING_TRAVEL_DEG;
    return ESP_OK;
}

static float steering_axis_to_duty(int32_t steering_axis)
{
    // Measured: this pad rests at axis=39, not 0, and the stock code maps that
    // straight through to duty 0.072 -- a continuous steering command with
    // nobody touching the controller. Ignore anything inside the deadband.
    if (steering_axis > -STEERING_DEADBAND && steering_axis < STEERING_DEADBAND) {
        return 0.0f;
    }

    if (steering_axis > (int32_t)GAMEPAD_STEERING_MAX) {
        steering_axis = (int32_t)GAMEPAD_STEERING_MAX;
    } else if (steering_axis < -(int32_t)GAMEPAD_STEERING_MAX) {
        steering_axis = -(int32_t)GAMEPAD_STEERING_MAX;
    }

    // Invert steering direction so controller left/right matches the vehicle direction.
    return ((float)steering_axis / GAMEPAD_STEERING_MAX) * STEERING_MAX_DUTY;
}

static float apply_steering_limit(float duty, int raw_adc)
{
    // Absolute end stops. Block only the direction that would travel FURTHER
    // past the stop; the recovering direction always stays available.
    if (raw_adc <= STEERING_ADC_LIMIT_A && duty > 0.0f) {
        return 0.0f;   // positive duty lowers the count, deeper into stop A
    }
    if (raw_adc >= STEERING_ADC_LIMIT_B && duty < 0.0f) {
        return 0.0f;
    }
    return duty;
}

static esp_err_t mcp4725_attach(uint8_t address)
{
    const i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    if (s_mcp4725 != NULL) {
        ESP_RETURN_ON_ERROR(i2c_master_bus_rm_device(s_mcp4725), TAG, "i2c_master_bus_rm_device failed");
        s_mcp4725 = NULL;
    }

    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_i2c_bus, &dev_config, &s_mcp4725), TAG, "add mcp4725 failed");
    s_mcp4725_address = address;
    return ESP_OK;
}

static bool mcp4725_present_and_attach(void)
{
    int timeout_count = 0;
    int nack_count = 0;
    const TickType_t probe_timeout = pdMS_TO_TICKS(300);

    for (uint8_t addr = MCP4725_ADDR_START; addr <= MCP4725_ADDR_END; ++addr) {
        for (int attempt = 0; attempt < 3; ++attempt) {
            const esp_err_t ret = i2c_master_probe(s_i2c_bus, addr, probe_timeout);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "I2C device detected at 0x%02X", addr);
                return mcp4725_attach(addr) == ESP_OK;
            }

            if (ret == ESP_ERR_TIMEOUT) {
                timeout_count++;
            } else {
                nack_count++;
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }

    ESP_LOGW(TAG,
             "I2C probe summary: timeout=%d nack_or_other=%d sda=%d scl=%d",
             timeout_count,
             nack_count,
             gpio_get_level(SDA_PIN),
             gpio_get_level(SCL_PIN));
    return false;
}

static esp_err_t set_dac_voltage(float requested_voltage)
{
    if (!s_dac_available || s_mcp4725 == NULL) {
        // No throttle hardware. Writing 0 V to a DAC that is not there succeeded
        // in the only sense that matters: nothing moved. Report OK so callers
        // wrapped in ESP_ERROR_CHECK() do not abort. Anything that needs to know
        // whether throttle exists must test s_dac_available, not this return.
        return ESP_OK;
    }
    const float max_voltage = DAC_SUPPLY_VOLTAGE * 4095.0f / 4096.0f;
    if (requested_voltage < 0.0f) {
        requested_voltage = 0.0f;
    } else if (requested_voltage > max_voltage) {
        requested_voltage = max_voltage;
    }

    uint16_t code = (uint16_t)lroundf((requested_voltage / DAC_SUPPLY_VOLTAGE) * 4096.0f);
    if (code > 4095) {
        code = 4095;
    }

    uint8_t payload[3] = {
        MCP4725_CMD_WRITE_DAC,
        (uint8_t)(code >> 4),
        (uint8_t)((code & 0x0F) << 4),
    };

    return i2c_master_transmit(s_mcp4725, payload, sizeof(payload), pdMS_TO_TICKS(100));
}

static float throttle_to_voltage(int32_t throttle)
{
    const float dac_max_voltage = DAC_SUPPLY_VOLTAGE * 4095.0f / 4096.0f;
    const float motor_start_voltage = 1.14f;
    const float speed_cap_voltage = 1.66f;  // ~4 m/s cap from field calibration.
    const float trigger_start_fraction = 0.25f;

    float throttle_max_voltage = speed_cap_voltage;
    if (throttle_max_voltage > dac_max_voltage) {
        throttle_max_voltage = dac_max_voltage;
    }

    if (throttle < 0) {
        throttle = 0;
    } else if (throttle > (int32_t)GAMEPAD_TRIGGER_MAX) {
        throttle = (int32_t)GAMEPAD_TRIGGER_MAX;
    }

    if (throttle == 0) {
        return 0.0f;
    }

    // Normalize throttle to 0.0-1.0
    const float normalized = throttle / GAMEPAD_TRIGGER_MAX;

    // Stretch the trigger travel so the car only begins to move around 25% RT press.
    if (normalized <= trigger_start_fraction) {
        return (normalized / trigger_start_fraction) * motor_start_voltage;
    }

    const float post_start = (normalized - trigger_start_fraction) / (1.0f - trigger_start_fraction);

    // Linear ramp after motor engagement keeps low-speed modulation predictable.
    return motor_start_voltage + (post_start * (throttle_max_voltage - motor_start_voltage));
}

static void dac_task(void *arg)
{
    ARG_UNUSED(arg);

    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_log = 0;
    TickType_t last_heartbeat = 0;
    float last_voltage = -1.0f;

    while (1) {
        // PRESENCE IS THE BLUETOOTH LINK STATE, AND NOTHING ELSE.
        //
        // s_controller_connected is set and cleared ONLY by Bluepad32's connect
        // and disconnect callbacks, so it is the one authoritative answer to
        // "is the operator's pad there".
        //
        // A staleness term used to be ANDed in here. It cannot work: this pad
        // transmits only on change, so an idle pad is silent INDEFINITELY and
        // any finite timeout eventually fires. At 100 ms it made AUTO
        // impossible to engage -- the transition needs the pad connected AND
        // centred, and centred means idle means silent. Raised to 10 s on
        // 2026-09-08 it merely moved the problem: the pad was then measured
        // dropping at exactly 10.2 s of idle. An output-report keep-alive does
        // not help either, because s_last_ctl_us is only advanced by INPUT
        // reports from the pad.
        //
        // What still protects against a wedged BTstack that never fires a
        // disconnect: the Jetson's own 200 ms $C watchdog, and the operator.
        const bool connected = s_controller_connected;
        const int32_t throttle = connected ? s_latest_throttle : 0;
        const int32_t steering_axis = connected ? s_latest_steering : 0;
        // Jetson throttle authority (v2), gated the same way as the brake.
        // Throttle is the only actuator that ADDS energy, so it is the last
        // one enabled and the first one dropped.
        int32_t throttle_source = throttle;
        float jetson_throttle = 0.0f;
        if (jetson_link_throttle_override(&jetson_throttle)) {
            throttle_source = (int32_t)(jetson_throttle * GAMEPAD_TRIGGER_MAX);
        }
        float target_voltage = throttle_to_voltage(throttle_source);
        // Cruise is a MANUAL-ONLY TEST AID for mapping throttle voltage to real
        // speed outdoors, where the operator is on the kart and cannot reach a
        // keyboard. It is a FLOOR, so the trigger always wins if it asks for
        // more.
        //
        // GATED TO MANUAL DELIBERATELY. A first version applied it in every
        // mode, which would have let a latched setpoint override Autoware
        // asking for ZERO throttle during autonomy. In AUTO the Jetson is the
        // only source of throttle, full stop.
        //
        // Also requires the pad present: losing it zeroes throttle anyway via
        // the disconnect callback.
        if (jetson_link_mode() == JLINK_MANUAL && connected &&
            s_cruise_v > target_voltage) {
            target_voltage = s_cruise_v;
        }
        float steering_angle_deg = NAN;

        if (steering_read_relative_angle(&steering_angle_deg) != ESP_OK) {
            // Fail CLOSED. The stock code substituted 0.0 here, which sits
            // inside the +/-80 window and therefore silently disabled the end
            // stop -- an ADC fault removed the only protection the steering has.
            steering_angle_deg = NAN;
        }

        float steering_duty = steering_axis_to_duty(steering_axis);
        if (isfinite(steering_angle_deg)) {
            steering_duty = apply_steering_limit(steering_duty, s_steering_raw);
        } else {
            steering_duty = 0.0f;
        }

        if (!connected) {
            steering_duty = 0.0f;
        }

        // ---- BRAKE ----------------------------------------------------
        // Differences from the brake-integration original, all for the same
        // reason the steering needed them: an ignored return value or a railed
        // pot silently becomes a command.
        const int32_t brake_trigger = connected ? s_latest_brake : 0;
        int raw_brake_adc = 0;
        const esp_err_t brake_adc_ret =
            adc_oneshot_read(s_adc, BRAKE_ADC_CHANNEL, &raw_brake_adc);

        if (brake_adc_ret != ESP_OK ||
            raw_brake_adc <= BRAKE_ADC_RAIL_LO || raw_brake_adc >= BRAKE_ADC_RAIL_HI) {
            // The original ignored the return value entirely and would drive on
            // a stale or zero reading. A railed pot means the feedback is gone,
            // and bang-bang against no feedback runs the actuator into a stop.
            s_brake_raw = raw_brake_adc;
            if (!s_brake_fault) {
                ESP_LOGE(TAG, "brake pot implausible (raw=%d) -- disabling brake driver",
                         raw_brake_adc);
                s_brake_fault = true;
            }
            brake_disable();
        } else {
            s_brake_raw = raw_brake_adc;
            if (s_brake_fault) {
                ESP_LOGI(TAG, "brake pot recovered");
                s_brake_fault = false;
                gpio_set_level(BRAKE_EN_PIN, 1);
            }
            if (s_smoothed_brake_adc < 0.0f) {
                s_smoothed_brake_adc = (float)raw_brake_adc;
            }
            s_smoothed_brake_adc = (BRAKE_FILTER_ALPHA * raw_brake_adc) +
                                   ((1.0f - BRAKE_FILTER_ALPHA) * s_smoothed_brake_adc);
            const int current_brake_adc = (int)s_smoothed_brake_adc;

            // Jetson brake authority (v2). Takes precedence over the pad
            // trigger ONLY while in AUTO with a fresh command and the
            // BRAKE_AUTH flag set; otherwise the pad's value stands.
            int32_t brake_source = brake_trigger;
            float jetson_brake = 0.0f;
            if (jetson_link_brake_override(&jetson_brake)) {
                brake_source = (int32_t)(jetson_brake * 1023.0f);
            }
            int target_brake_adc = map_val(brake_source, 0, 1023,
                                           BRAKE_ADC_MIN, BRAKE_ADC_MAX);
            if (!connected) {
                // FAIL DIRECTION -- see 16_brake_integration_review.md.
                // This RELEASES the brake on loss of the controller, which is
                // the original behaviour and is NOT obviously right for a
                // vehicle. Deliberately left as-is pending the datasheet
                // answer, but it is a decision, not a default.
                target_brake_adc = BRAKE_ADC_MIN;
            }
            if (target_brake_adc < BRAKE_ADC_MIN) target_brake_adc = BRAKE_ADC_MIN;
            if (target_brake_adc > BRAKE_ADC_MAX) target_brake_adc = BRAKE_ADC_MAX;

            const int brake_error = target_brake_adc - current_brake_adc;
            const int brake_mag = abs(brake_error);
            if (brake_mag > BRAKE_DEADBAND) {
                const int pwm = (brake_mag > BRAKE_NEAR_BAND) ? BRAKE_SPEED_PWM
                                                              : BRAKE_SPEED_PWM_NEAR;
                if (brake_error > 0) brake_extend(pwm);
                else brake_retract(pwm);
            } else {
                brake_stop();
            }
        }

        // MANUAL passes the pad through unchanged; AUTO substitutes the Jetson
        // setpoint; FAULT forces zero. The pad always wins.
        const bool pad_centred = (labs((long)steering_axis) <= STEERING_DEADBAND);
        steering_duty = jetson_link_arbitrate(steering_duty, connected,
                                              pad_centred, steering_angle_deg);

        // END STOPS ARE UNCONDITIONAL. They were previously applied only to the
        // MANUAL duty further up, and then jetson_link_arbitrate() OVERWROTE
        // that value in AUTO -- so autonomy bypassed the limits entirely. On
        // 2026-09-08 a sign-inverted angle loop ran the steering to ADC 789,
        // well past the 924 limit, with nothing to stop it.
        //
        // Applying it here, on the FINAL duty whatever mode produced it, is the
        // only version that is actually a limit. Keep this the LAST thing that
        // touches steering_duty before it goes on the wire.
        steering_duty = apply_steering_limit(steering_duty, s_steering_raw);

        if (last_voltage < 0.0f || fabsf(target_voltage - last_voltage) >= 0.01f) {
            const esp_err_t ret = set_dac_voltage(target_voltage);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "DAC write failed: %s", esp_err_to_name(ret));
            } else {
                last_voltage = target_voltage;
            }
        }

        const TickType_t now = xTaskGetTickCount();
        if (now - last_heartbeat >= pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS)) {
            last_heartbeat = now;
            if (steering_can_ensure_ready()) {
                const esp_err_t hb_ret = steering_can_send_heartbeat();
                if (hb_ret != ESP_OK) {
                    ESP_LOGW(TAG, "CAN heartbeat failed: %s", esp_err_to_name(hb_ret));
                    log_twai_status("heartbeat");
                }

                const esp_err_t duty_ret = steering_can_send_duty(steering_duty);
                if (duty_ret != ESP_OK) {
                    ESP_LOGW(TAG, "CAN steering duty failed: %s", esp_err_to_name(duty_ret));
                    log_twai_status("duty");
                }
            }
        }

        jetson_link_publish(steering_angle_deg, steering_duty, steering_axis,
                            throttle, connected, s_steering_raw,
                            s_brake_raw, brake_trigger, s_brake_fault,
                            s_cruise_v);

        // The periodic status line used to live here. It was written with
        // ESP_LOGI, which reaches UART0 via stdout/VFS, while jetson_link
        // writes $T with uart_write_bytes directly -- two paths to one UART
        // that interleave at the FIFO and truncate each other mid-line.
        // $T now carries every field the log line had, atomically, at 50 Hz.
        (void)last_log;

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(DAC_UPDATE_PERIOD_MS));
    }
}

void xbox_dac_on_controller_connected(bool connected)
{
    s_controller_connected = connected;
    if (!connected) {
        s_latest_throttle = 0;
        s_latest_steering = 0;
        s_latest_brake = 0;
        s_prev_buttons = 0;
    }
}

void xbox_dac_note_controller_alive(void)
{
    s_last_ctl_us = esp_timer_get_time();
}

void xbox_dac_on_controller_data(const uni_controller_t* ctl)
{
    if (ctl == NULL || ctl->klass != UNI_CONTROLLER_CLASS_GAMEPAD) {
        return;
    }

    s_latest_throttle = ctl->gamepad.throttle;
    s_latest_steering = ctl->gamepad.axis_x;
    s_latest_brake = ctl->gamepad.brake;      // LT trigger, 0..1023
    s_last_ctl_us = esp_timer_get_time();

    const uint32_t buttons = ctl->gamepad.buttons;
    const bool x_pressed = (buttons & BUTTON_X) != 0;
    const bool x_was_pressed = (s_prev_buttons & BUTTON_X) != 0;
    const bool y_now = (buttons & BUTTON_Y) != 0;
    const bool y_was = (s_prev_buttons & BUTTON_Y) != 0;
    const bool a_now = (buttons & BUTTON_A) != 0;
    const bool a_was = (s_prev_buttons & BUTTON_A) != 0;
    const bool b_now = (buttons & BUTTON_B) != 0;
    const bool b_was = (s_prev_buttons & BUTTON_B) != 0;

    if (y_now && !y_was) {
        s_cruise_v += CRUISE_STEP_V;
        if (s_cruise_v > CRUISE_MAX_V) s_cruise_v = CRUISE_MAX_V;
        ESP_LOGI(TAG, "cruise %+.2f -> %.2f V", CRUISE_STEP_V, s_cruise_v);
    }
    if (a_now && !a_was) {
        s_cruise_v -= CRUISE_STEP_V;
        if (s_cruise_v < 0.0f) s_cruise_v = 0.0f;
        ESP_LOGI(TAG, "cruise %+.2f -> %.2f V", -CRUISE_STEP_V, s_cruise_v);
    }
    if (b_now && !b_was) {
        s_cruise_v = 0.0f;
        ESP_LOGW(TAG, "cruise CLEARED by B");
    }

    if (x_pressed && !x_was_pressed) {
        // The X button used to re-zero the steering at the current position.
        // That is exactly the boot-relative behaviour removed on 2026-09-08:
        // the zero is now the MEASURED absolute STEERING_ADC_ZERO, so there is
        // nothing to reset and pressing X must not move the reference. Kept as
        // a log line only, so an old habit does not silently do nothing.
        ESP_LOGW(TAG, "X pressed: steering zero is now ABSOLUTE "
                      "(STEERING_ADC_ZERO=%d) and cannot be re-zeroed here",
                 STEERING_ADC_ZERO);
    }
    s_prev_buttons = buttons;
}

void app_main(void)
{
    // Try normal pin order first, then swapped, to survive mislabelled perfboard traces.
    const struct { int sda; int scl; } pin_candidates[] = {
        { SDA_PIN, SCL_PIN },
        { SCL_PIN, SDA_PIN },
    };
    bool dac_found = false;
    for (int c = 0; c < (int)(sizeof(pin_candidates) / sizeof(pin_candidates[0])); ++c) {
        int sda = pin_candidates[c].sda;
        int scl = pin_candidates[c].scl;
        ESP_LOGI(TAG, "Trying I2C SDA=GPIO%d SCL=GPIO%d", sda, scl);
        if (i2c_master_init_pins(sda, scl) != ESP_OK) {
            ESP_LOGW(TAG, "Bus init failed for SDA=%d SCL=%d", sda, scl);
            continue;
        }
        if (mcp4725_present_and_attach()) {
            ESP_LOGI(TAG, "MCP4725 found with SDA=GPIO%d SCL=GPIO%d", sda, scl);
            dac_found = true;
            break;
        }
    }
    if (!dac_found) {
        ESP_LOGE(TAG, "MCP4725 not found on any pin combination. Check VCC/GND/SDA/SCL and pull-ups.");
        ESP_LOGW(TAG, "CONTINUING WITHOUT THROTTLE. Steering, brake, CAN and telemetry still run.");
        s_dac_available = false;
    } else {
        s_dac_available = true;
    }

    ESP_ERROR_CHECK(steering_adc_init());
    ESP_ERROR_CHECK(brake_init());
    ESP_ERROR_CHECK(steering_can_init());
    vTaskDelay(pdMS_TO_TICKS(100));

    if (!mcp4725_present_and_attach()) {
        ESP_LOGE(TAG, "MCP4725 not found on 0x60..0x67. Check VCC/GND/SDA/SCL and pull-ups.");
        ESP_LOGW(TAG, "THROTTLE DISABLED. All other subsystems continue.");
        s_dac_available = false;
    } else {
        s_dac_available = true;
        ESP_LOGI(TAG, "MCP4725 attached at 0x%02X", s_mcp4725_address);
    }
    ESP_ERROR_CHECK(set_dac_voltage(0.0f));
    ESP_ERROR_CHECK(steering_can_send_heartbeat());
    ESP_ERROR_CHECK(steering_can_send_duty(0.0f));

    jetson_link_start();

#if CONFIG_FREERTOS_UNICORE
    xTaskCreate(dac_task, "dac_task", 4096, NULL, 5, NULL);
#else
    xTaskCreatePinnedToCore(dac_task, "dac_task", 4096, NULL, 5, NULL, 1);
#endif

#ifdef CONFIG_ESP_CONSOLE_UART
#ifndef CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
    // Skipped when the Jetson link shares UART0: btstack_stdio_init() installs
    // its own driver on the console UART and would fight jetson_link_start().
#if JLINK_SHARES_CONSOLE == 0
    btstack_stdio_init();
#endif
#endif
#endif

    btstack_init();
    uni_platform_set_custom(get_my_platform());
    uni_init(0, NULL);
    btstack_run_loop_execute();
}
