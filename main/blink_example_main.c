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
#include "driver/i2c_master.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcpp"
#include "driver/twai.h"
#pragma GCC diagnostic pop
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "xbox_dac";

#define I2C_PORT I2C_NUM_0
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_FREQ_HZ 100000

#define MCP4725_ADDR_START 0x60
#define MCP4725_ADDR_END 0x67
#define MCP4725_CMD_WRITE_DAC 0x40

#define DAC_SUPPLY_VOLTAGE 3.30f
#define GAMEPAD_TRIGGER_MAX 1023.0f
#define GAMEPAD_STEERING_MAX 512.0f
#define DAC_UPDATE_PERIOD_MS 20
#define DAC_LOG_PERIOD_MS 500

#define STEERING_ADC_CHANNEL ADC_CHANNEL_6  // GPIO34
#define STEERING_SENSOR_MIN_V 0.30f
#define STEERING_SENSOR_MAX_V 4.77f
#define STEERING_LIMIT_DEG 80.0f
#define STEERING_MAX_DUTY 0.95f

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
static volatile int32_t s_latest_throttle = 0;
static volatile int32_t s_latest_steering = 0;
static volatile bool s_controller_connected = false;
static float s_steering_zero_offset_deg = NAN;
static bool s_twai_recovering = false;

struct uni_platform* get_my_platform(void);

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

static esp_err_t i2c_master_init(void)
{
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &s_i2c_bus), TAG, "i2c_new_master_bus failed");
    return ESP_OK;
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
                        "adc_oneshot_config_channel failed");
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

static esp_err_t steering_read_relative_angle(float* relative_angle_deg)
{
    int raw = 0;
    ESP_RETURN_ON_ERROR(adc_oneshot_read(s_adc, STEERING_ADC_CHANNEL, &raw), TAG, "adc_oneshot_read failed");

    float esp_volts = ((float)raw / 4095.0f) * 3.3f;
    float sensor_volts = esp_volts * 2.0f;  // 10k/10k divider

    const float voltage_range = STEERING_SENSOR_MAX_V - STEERING_SENSOR_MIN_V;
    float absolute_angle = (sensor_volts - STEERING_SENSOR_MIN_V) * (360.0f / voltage_range);
    if (absolute_angle < 0.0f) {
        absolute_angle = 0.0f;
    } else if (absolute_angle > 360.0f) {
        absolute_angle = 360.0f;
    }

    if (isnan(s_steering_zero_offset_deg)) {
        s_steering_zero_offset_deg = absolute_angle;
    }

    float relative = absolute_angle - s_steering_zero_offset_deg;
    if (relative > 180.0f) {
        relative -= 360.0f;
    } else if (relative < -180.0f) {
        relative += 360.0f;
    }

    *relative_angle_deg = relative;
    return ESP_OK;
}

static float steering_axis_to_duty(int32_t steering_axis)
{
    if (steering_axis > (int32_t)GAMEPAD_STEERING_MAX) {
        steering_axis = (int32_t)GAMEPAD_STEERING_MAX;
    } else if (steering_axis < -(int32_t)GAMEPAD_STEERING_MAX) {
        steering_axis = -(int32_t)GAMEPAD_STEERING_MAX;
    }

    // Negative sign keeps steering direction aligned with existing Teensy behavior.
    return -((float)steering_axis / GAMEPAD_STEERING_MAX) * STEERING_MAX_DUTY;
}

static float apply_steering_limit(float duty, float relative_angle_deg)
{
    if (relative_angle_deg >= STEERING_LIMIT_DEG && duty < 0.0f) {
        return 0.0f;
    }
    if (relative_angle_deg <= -STEERING_LIMIT_DEG && duty > 0.0f) {
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
    for (uint8_t addr = MCP4725_ADDR_START; addr <= MCP4725_ADDR_END; ++addr) {
        const esp_err_t ret = i2c_master_probe(s_i2c_bus, addr, pdMS_TO_TICKS(100));
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "I2C device detected at 0x%02X", addr);
            return mcp4725_attach(addr) == ESP_OK;
        }
    }
    return false;
}

static esp_err_t set_dac_voltage(float requested_voltage)
{
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
    const float max_voltage = DAC_SUPPLY_VOLTAGE * 4095.0f / 4096.0f;
    const float dead_zone_voltage = 0.72f;  // Motor starts spinning voltage

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
    
    // Use a square root curve for responsive acceleration
    // This gives you quick motor start with light trigger press,
    // while maintaining smooth control across the full range.
    // - 10% press (trigger ~102):   reaches ~1.2V (motor moving smoothly)
    // - 25% press (trigger ~256):   reaches ~1.9V (good acceleration)
    // - 50% press (trigger ~512):   reaches ~2.5V (strong response)
    // - 100% press (trigger ~1023): reaches ~3.3V (maximum)
    const float curved = sqrtf(normalized);
    
    // Map curve output to voltage range
    float voltage = dead_zone_voltage + (curved * (max_voltage - dead_zone_voltage));
    
    return voltage;
}

static void dac_task(void *arg)
{
    ARG_UNUSED(arg);

    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_log = 0;
    TickType_t last_heartbeat = 0;
    float last_voltage = -1.0f;

    while (1) {
        const bool connected = s_controller_connected;
        const int32_t throttle = connected ? s_latest_throttle : 0;
        const int32_t steering_axis = connected ? s_latest_steering : 0;
        const float target_voltage = throttle_to_voltage(throttle);
        float steering_angle_deg = 0.0f;

        if (steering_read_relative_angle(&steering_angle_deg) != ESP_OK) {
            steering_angle_deg = 0.0f;
        }

        float steering_duty = steering_axis_to_duty(steering_axis);
        steering_duty = apply_steering_limit(steering_duty, steering_angle_deg);

        if (!connected) {
            steering_duty = 0.0f;
        }

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

        if (now - last_log >= pdMS_TO_TICKS(DAC_LOG_PERIOD_MS)) {
            last_log = now;
            ESP_LOGI(TAG,
                     "controller=%s throttle=%ld voltage=%.3fV steer_axis=%ld angle=%.1f duty=%.3f",
                     connected ? "connected" : "waiting", (long)throttle, target_voltage,
                     (long)steering_axis, steering_angle_deg, steering_duty);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(DAC_UPDATE_PERIOD_MS));
    }
}

void xbox_dac_on_controller_connected(bool connected)
{
    s_controller_connected = connected;
    if (!connected) {
        s_latest_throttle = 0;
    }
}

void xbox_dac_on_controller_data(const uni_controller_t* ctl)
{
    if (ctl == NULL || ctl->klass != UNI_CONTROLLER_CLASS_GAMEPAD) {
        return;
    }

    s_latest_throttle = ctl->gamepad.throttle;
    s_latest_steering = ctl->gamepad.axis_x;
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_ERROR_CHECK(steering_adc_init());
    ESP_ERROR_CHECK(steering_can_init());
    vTaskDelay(pdMS_TO_TICKS(100));

    if (!mcp4725_present_and_attach()) {
        ESP_LOGE(TAG, "MCP4725 not found on 0x60..0x67. Check VCC/GND/SDA/SCL and pull-ups.");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "MCP4725 attached at 0x%02X", s_mcp4725_address);
    ESP_ERROR_CHECK(set_dac_voltage(0.0f));
    ESP_ERROR_CHECK(steering_can_send_heartbeat());
    ESP_ERROR_CHECK(steering_can_send_duty(0.0f));

#if CONFIG_FREERTOS_UNICORE
    xTaskCreate(dac_task, "dac_task", 4096, NULL, 5, NULL);
#else
    xTaskCreatePinnedToCore(dac_task, "dac_task", 4096, NULL, 5, NULL, 1);
#endif

#ifdef CONFIG_ESP_CONSOLE_UART
#ifndef CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
    btstack_stdio_init();
#endif
#endif

    btstack_init();
    uni_platform_set_custom(get_my_platform());
    uni_init(0, NULL);
    btstack_run_loop_execute();
}
