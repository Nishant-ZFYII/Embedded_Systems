#include "jetson_link.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "jlink";

#define JLINK_RX_BUF 1024
#define JLINK_LINE_MAX 160

#define JLINK_CMD_TIMEOUT_MS 200    // 10 missed frames at 50 Hz
#define JLINK_ARM_FRAMES 10         // consecutive good frames before AUTO
#define JLINK_CHECKSUM_LIMIT 5      // bad frames in one second -> FAULT
#define JLINK_RAMP_MS 250           // blend time on every mode transition
#define JLINK_SLEW_PER_S 4.0f       // max |d(duty)|/dt in AUTO
#define JLINK_MAX_AUTO_DUTY 0.95f   // never exceed the pad's own clamp

// ---- steering angle loop (v2) -------------------------------------------
// PLACEHOLDER GAINS. Tuned on the bench, never on the ground.
//
// Breakaway is MEASURED at duty 0.45-0.60 and it is SHARP: 2.4 deg of travel at
// 0.45 versus 41.5 deg at 0.60. A plain proportional term therefore stalls --
// it produces small duties that energise the motor and move nothing. The
// feed-forward term below carries the output to the edge of breakaway as soon
// as the error is meaningful, and the integrator climbs past it.
#define JLINK_ANGLE_KP        0.020f   // duty per degree of error
#define JLINK_ANGLE_KI        0.010f   // duty per degree-second
#define JLINK_ANGLE_FF        0.42f    // just under measured breakaway
#define JLINK_ANGLE_DEADBAND  1.0f     // deg; sensor noise is +/-0.13
#define JLINK_ANGLE_I_CLAMP   0.35f
#define JLINK_ANGLE_MAX_DUTY  0.75f    // below the pad's 0.95 while unproven

// ---------------------------------------------------------------------------
static volatile jlink_mode_t s_mode = JLINK_MANUAL;
static volatile jlink_fault_t s_fault = JLINK_FAULT_NONE;
static volatile bool s_estop = false;      // operator E-STOP, latched

static volatile float s_cmd_duty = 0.0f;      // last accepted Jetson setpoint
static volatile float s_cmd_throttle = 0.0f;  // 0..1
static volatile float s_cmd_brake = 0.0f;     // 0..1
static volatile uint8_t s_cmd_steer_mode = JLINK_STEER_MODE_DUTY;
static volatile bool s_cmd_brake_auth = false;
static volatile bool s_cmd_throttle_auth = false;
static float s_angle_integral = 0.0f;
static volatile uint8_t s_cmd_seq = 0;
static volatile bool s_cmd_arm = false;
static volatile bool s_cmd_want_auto = false;
static volatile int64_t s_cmd_us = 0;         // arrival time of that setpoint

static volatile uint32_t s_good_frames = 0;
static volatile uint32_t s_crc_errors = 0;
static volatile bool s_clear_request = false;

static float s_applied = 0.0f;                // what we returned last cycle
static float s_ramp_from = 0.0f;
static int64_t s_ramp_start_us = 0;
static bool s_ramping = false;

static inline int64_t now_us(void) { return esp_timer_get_time(); }

static uint8_t nmea_xor(const char *s, int len)
{
    uint8_t c = 0;
    for (int i = 0; i < len; ++i) c ^= (uint8_t)s[i];
    return c;
}

// ---------------------------------------------------------------------------
// Parsing.  $C,<seq>,<mode>,<steer>,<brake>,<flags>*<XX>
//   flags bit0 = ARM, bit1 = request AUTO
// Also accepts:
//   $X*<XX>   clear a latched fault
//
// Deliberately tolerant of leading noise: ESP_LOG output shares this line when
// JLINK_UART_NUM is 0, so anything before '$' is discarded rather than treated
// as an error.
// ---------------------------------------------------------------------------
static void handle_line(char *line, int len)
{
    char *dollar = memchr(line, '$', len);
    if (!dollar) return;                       // pure log noise, not an error
    len -= (int)(dollar - line);
    line = dollar;

    char *star = memchr(line, '*', len);
    if (!star || (star - line) < 2 || (len - (star - line)) < 3) {
        s_crc_errors++;
        return;
    }

    unsigned want = 0;
    if (sscanf(star + 1, "%2x", &want) != 1) { s_crc_errors++; return; }
    if (nmea_xor(line + 1, (int)(star - line - 1)) != (uint8_t)want) {
        s_crc_errors++;
        return;
    }
    *star = '\0';

    if (line[1] == 'X') {                      // clear fault
        s_clear_request = true;
        return;
    }
    if (line[1] != 'C') return;

    unsigned seq = 0, mode = 0, flags = 0;
    float steer = 0.0f, throttle = 0.0f, brake = 0.0f;

    // v2: $C,seq,mode,steer,throttle,brake,flags   (6 values)
    // v1: $C,seq,mode,steer,brake,flags            (5 values) -- still accepted
    int n = sscanf(line, "$C,%u,%u,%f,%f,%f,%u",
                   &seq, &mode, &steer, &throttle, &brake, &flags);
    if (n != 6) {
        throttle = 0.0f;
        n = sscanf(line, "$C,%u,%u,%f,%f,%u", &seq, &mode, &steer, &brake, &flags);
        if (n != 5) { s_crc_errors++; return; }
    }

    if (!isfinite(steer) || !isfinite(throttle) || !isfinite(brake)) {
        s_crc_errors++;
        return;
    }

    if (mode == JLINK_STEER_MODE_ANGLE) {
        // radians of tire angle; the loop clamps against the end stops
        s_cmd_steer_mode = JLINK_STEER_MODE_ANGLE;
    } else {
        s_cmd_steer_mode = JLINK_STEER_MODE_DUTY;
        if (steer > JLINK_MAX_AUTO_DUTY) steer = JLINK_MAX_AUTO_DUTY;
        if (steer < -JLINK_MAX_AUTO_DUTY) steer = -JLINK_MAX_AUTO_DUTY;
    }

    if (throttle < 0.0f) throttle = 0.0f;
    if (throttle > 1.0f) throttle = 1.0f;
    if (brake < 0.0f) brake = 0.0f;
    if (brake > 1.0f) brake = 1.0f;

    s_cmd_throttle = throttle;
    s_cmd_brake = brake;
    s_cmd_brake_auth = (flags & JLINK_FLAG_BRAKE_AUTH) != 0;
    s_cmd_throttle_auth = (flags & JLINK_FLAG_THROTTLE_AUTH) != 0;
    s_cmd_duty = steer;
    s_cmd_seq = (uint8_t)seq;
    s_cmd_arm = (flags & JLINK_FLAG_ARM) != 0;
    s_cmd_want_auto = (flags & JLINK_FLAG_AUTO) != 0;
    s_cmd_us = now_us();
    if (s_good_frames < 0xFFFF) s_good_frames++;
}

static void jlink_rx_task(void *arg)
{
    (void)arg;
    static char line[JLINK_LINE_MAX];
    int fill = 0;
    uint8_t buf[128];

    int64_t window_us = now_us();
    uint32_t window_errs = 0;

    while (1) {
        int n = uart_read_bytes(JLINK_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(20));
        for (int i = 0; i < n; ++i) {
            char c = (char)buf[i];
            if (c == '\n' || c == '\r') {
                if (fill > 0) {
                    line[fill] = '\0';     // sscanf below needs termination
                    handle_line(line, fill);
                    fill = 0;
                }
            } else if (fill < JLINK_LINE_MAX - 1) {
                line[fill++] = c;
            } else {
                fill = 0;                       // overlong: drop it
                s_crc_errors++;
            }
        }

        // checksum error rate over a one second window
        uint32_t errs = s_crc_errors;
        if (now_us() - window_us > 1000000) {
            window_us = now_us();
            window_errs = errs;
        }
        if (errs - window_errs >= JLINK_CHECKSUM_LIMIT && s_mode == JLINK_AUTO) {
            s_fault = JLINK_FAULT_CHECKSUM;
            s_mode = JLINK_FAULT;
        }
    }
}

void jetson_link_start(void)
{
    const uart_config_t cfg = {
        .baud_rate = JLINK_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(JLINK_UART_NUM, JLINK_RX_BUF, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(JLINK_UART_NUM, &cfg));
#if JLINK_UART_NUM != 0
    ESP_ERROR_CHECK(uart_set_pin(JLINK_UART_NUM, JLINK_TX_GPIO, JLINK_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
#endif
    // PINNED TO CORE 1, AWAY FROM BLUETOOTH.
    //
    // This was xTaskCreate(), i.e. no core affinity, so it could run on core 0 --
    // where CONFIG_BTDM_CTRL_PINNED_TO_CORE=0 puts the Bluetooth controller. At
    // priority 6 it wakes 50 times a second the moment the Jetson link is up, and
    // starved the BT controller until the link supervision timer expired. BTstack
    // then fired a REAL disconnect, which is why raising CONTROLLER_STALE_US did
    // not help: s_controller_connected was genuinely false.
    //
    // Evidence (2026-09-08): the pad survives HOURS on joystick control alone,
    // where jlink_rx is idle, but dropped within ~4 s of the Jetson engaging
    // AUTO. Meanwhile the main loop (dac_task, pinned to core 1) held 20.2 ms
    // with sub-ms jitter straight through every dropout -- core 1 was fine, core
    // 0 was being squeezed. That measurement was initially misread as proof the
    // ESP32 was healthy; it only ever proved core 1 was.
    //
    // Core 1 already runs dac_task at priority 5. jlink_rx at 6 preempts it
    // briefly, which that loop has ample slack to absorb.
    xTaskCreatePinnedToCore(jlink_rx_task, "jlink_rx", 4096, NULL, 6, NULL, 1);
    ESP_LOGI(TAG, "link up on UART%d @ %d, MANUAL (rx task on core 1)",
             JLINK_UART_NUM, JLINK_BAUD);
}

jlink_mode_t jetson_link_mode(void) { return s_mode; }

// ---- longitudinal authority ---------------------------------------------
// Both return false unless we are actually in AUTO, the command is fresh, and
// the Jetson has been granted authority for that specific actuator. Anything
// less and the pad's value stands. Staged deliberately: brake before throttle,
// because throttle is the only actuator that ADDS energy and the brake is its
// only mitigation.
bool jetson_link_brake_override(float* out_0_1)
{
    if (s_mode != JLINK_AUTO || !s_cmd_brake_auth) return false;
    if ((now_us() - s_cmd_us) >= (JLINK_CMD_TIMEOUT_MS * 1000)) return false;
    *out_0_1 = s_cmd_brake;
    return true;
}

bool jetson_link_throttle_override(float* out_0_1)
{
    if (s_mode != JLINK_AUTO || !s_cmd_throttle_auth) return false;
    if ((now_us() - s_cmd_us) >= (JLINK_CMD_TIMEOUT_MS * 1000)) return false;
    *out_0_1 = s_cmd_throttle;
    return true;
}

void jetson_link_note_estop(bool latched)
{
    s_estop = latched;
    if (latched) {
        s_fault = JLINK_FAULT_ESTOP;
        s_mode = JLINK_FAULT;
    } else {
        // D-59, 2026-10-01. Clearing s_estop is NOT enough and this was wrong
        // when first written. The output gates live in two places: the
        // overrides in blink_example_main.c (released as soon as s_estop goes
        // false) and s_mode == JLINK_FAULT here, which forces duty to 0
        // regardless. Dropping only the first released the BRAKE and left the
        // STEERING dead -- exactly what the operator reported.
        //
        // FAULT is latched and, by rule 3, leaves only through the gate below,
        // which until now was reachable only by the Jetson sending $X. Raising
        // the same request here lets the pad clear its own e-stop while keeping
        // every condition that gate already enforces: pad centred, angle
        // finite, and now !s_estop. The pad cannot bypass those, it can only
        // ASK, which is the point.
        s_clear_request = true;
    }
}

void jetson_link_note_can_fault(void)
{
    if (s_mode == JLINK_AUTO) {
        s_fault = JLINK_FAULT_CAN;
        s_mode = JLINK_FAULT;
    }
}

// ---------------------------------------------------------------------------
// The arbiter.
//
// Rules, in priority order:
//   1. A failed angle read is a FAULT, and FAULT means zero duty. The stock
//      firmware substituted 0.0 degrees here, which is inside the +/-80 window
//      and therefore silently DISABLED the end stop. That was fail-open.
//   2. The pad always wins. Any deflection past the deadband drops AUTO
//      immediately, with no confirmation and no way for the Jetson to refuse.
//   3. FAULT is latched and clears only on an explicit $X plus a centred pad.
//   4. Every transition ramps over JLINK_RAMP_MS. The gearbox is 1:45 and a
//      step change snaps the wheel.
// ---------------------------------------------------------------------------
float jetson_link_arbitrate(float manual_duty, bool pad_active, bool pad_centred,
                            float angle_deg)
{
    const int64_t t = now_us();
    const bool cmd_fresh = (t - s_cmd_us) < (JLINK_CMD_TIMEOUT_MS * 1000);
    const jlink_mode_t prev = s_mode;

    if (!isfinite(angle_deg)) {                       // rule 1
        s_fault = JLINK_FAULT_ADC;
        s_mode = JLINK_FAULT;
    }

    if (s_clear_request) {
        s_clear_request = false;
        // E-STOP refuses this gate deliberately. Every other fault here clears
        // itself once the pad is centred and the sensor is sane -- which is
        // right for a transient, and completely wrong for a human who has just
        // hit the stop button. An E-STOP that un-latches on its own is not an
        // E-STOP.
        if (!s_estop && pad_centred && s_mode == JLINK_FAULT && isfinite(angle_deg)) {
            s_fault = JLINK_FAULT_NONE;
            s_mode = JLINK_MANUAL;
            ESP_LOGI(TAG, "fault cleared -> MANUAL");
        }
    }

    switch (s_mode) {
    case JLINK_MANUAL:
        if (s_cmd_arm && s_cmd_want_auto && cmd_fresh && pad_active &&
            pad_centred && s_good_frames >= JLINK_ARM_FRAMES) {
            s_mode = JLINK_AUTO;
            ESP_LOGI(TAG, "MANUAL -> AUTO");
        }
        break;

    case JLINK_AUTO:
        if (!pad_centred) {                            // rule 2
            s_mode = JLINK_MANUAL;
            ESP_LOGI(TAG, "pad moved -> MANUAL");
        } else if (!pad_active) {
            s_fault = JLINK_FAULT_PAD_LOST;
            s_mode = JLINK_FAULT;
            ESP_LOGW(TAG, "pad lost in AUTO -> FAULT");
        } else if (!cmd_fresh) {
            s_fault = JLINK_FAULT_TIMEOUT;
            s_mode = JLINK_FAULT;
            ESP_LOGW(TAG, "command timeout -> FAULT");
        } else if (!s_cmd_arm || !s_cmd_want_auto) {
            s_mode = JLINK_MANUAL;
            ESP_LOGI(TAG, "disarmed -> MANUAL");
        }
        break;

    case JLINK_FAULT:
        break;                                          // rule 3, latched
    }

    if (s_mode != prev) {                               // rule 4
        s_ramp_from = s_applied;
        s_ramp_start_us = t;
        s_ramping = true;
        s_good_frames = 0;
        s_angle_integral = 0.0f;
    }

    // In AUTO with steer mode ANGLE, close a loop on the measured shaft angle
    // instead of passing a duty straight through.
    float auto_duty = s_cmd_duty;
    if (s_mode == JLINK_AUTO && s_cmd_steer_mode == JLINK_STEER_MODE_ANGLE) {
        if (!isfinite(angle_deg)) {
            auto_duty = 0.0f;                       // handled as FAULT above
        } else {
            const float target_deg = s_cmd_duty * (180.0f / (float)M_PI);
            const float err = target_deg - angle_deg;

            if (fabsf(err) < JLINK_ANGLE_DEADBAND) {
                s_angle_integral = 0.0f;            // settled: do not creep
                auto_duty = 0.0f;
            } else {
                static int64_t ang_last_us = 0;
                float adt = ang_last_us ? (float)(t - ang_last_us) / 1e6f : 0.02f;
                if (adt <= 0.0f || adt > 0.1f) adt = 0.02f;
                ang_last_us = t;

                s_angle_integral += err * adt * JLINK_ANGLE_KI;
                if (s_angle_integral > JLINK_ANGLE_I_CLAMP)
                    s_angle_integral = JLINK_ANGLE_I_CLAMP;
                if (s_angle_integral < -JLINK_ANGLE_I_CLAMP)
                    s_angle_integral = -JLINK_ANGLE_I_CLAMP;

                // Feed-forward to the edge of measured breakaway. Without it a
                // proportional term produces duties that energise the motor and
                // move nothing -- static friction lets go abruptly at ~0.45-0.60.
                //
                // SIGN. POSITIVE DUTY DRIVES THE ADC COUNT DOWN, i.e. it makes
                // the reported angle MORE NEGATIVE (see STEERING_DUTY_POS_LOWERS
                // _COUNT in blink_example_main.c). So a positive error -- meaning
                // "measured is below target, drive the angle UP" -- needs a
                // NEGATIVE duty. The original sign was inverted, which made the
                // loop positive-feedback: measured 2026-09-08, commanding 0 deg
                // from -1.21 deg drove the steering to -110 deg and into the
                // mechanical lock. Every term is negated together.
                const float ff = (err > 0.0f ? -JLINK_ANGLE_FF : JLINK_ANGLE_FF);
                auto_duty = ff - err * JLINK_ANGLE_KP - s_angle_integral;

                if (auto_duty > JLINK_ANGLE_MAX_DUTY) auto_duty = JLINK_ANGLE_MAX_DUTY;
                if (auto_duty < -JLINK_ANGLE_MAX_DUTY) auto_duty = -JLINK_ANGLE_MAX_DUTY;
            }
        }
    } else {
        s_angle_integral = 0.0f;
    }

    float target;
    switch (s_mode) {
    case JLINK_AUTO:   target = auto_duty;   break;
    case JLINK_MANUAL: target = manual_duty; break;
    default:           target = 0.0f;        break;
    }

    if (s_ramping) {
        const float a = (float)(t - s_ramp_start_us) / (JLINK_RAMP_MS * 1000.0f);
        if (a >= 1.0f) s_ramping = false;
        else target = s_ramp_from + (target - s_ramp_from) * a;
    }

    // slew limit, AUTO only: MANUAL keeps the stock feel
    static int64_t last_us = 0;
    float dt = last_us ? (float)(t - last_us) / 1e6f : 0.02f;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.02f;   // first cycle, or a stall
    last_us = t;                               // advance in EVERY mode
    if (s_mode == JLINK_AUTO) {
        const float max_step = JLINK_SLEW_PER_S * dt;
        const float d = target - s_applied;
        if (d > max_step) target = s_applied + max_step;
        else if (d < -max_step) target = s_applied - max_step;
    }

    if (target > JLINK_MAX_AUTO_DUTY) target = JLINK_MAX_AUTO_DUTY;
    if (target < -JLINK_MAX_AUTO_DUTY) target = -JLINK_MAX_AUTO_DUTY;
    s_applied = target;
    return target;
}

void jetson_link_publish(float angle_deg, float applied_duty, int32_t axis,
                         int32_t throttle, bool pad_active, int raw_adc,
                         int brake_raw, int32_t brake_cmd, bool brake_fault,
                         float cruise_v, float dac_v)
{
    char body[JLINK_LINE_MAX];
    const int n = snprintf(body, sizeof(body),
                           "T,%u,%d,%.2f,%.4f,%ld,%ld,%u,%u,%u,%d,%d,%ld,%d,%.3f,%.3f",
                           (unsigned)s_cmd_seq, (int)s_mode,
                           isfinite(angle_deg) ? angle_deg : NAN,
                           applied_duty, (long)axis, (long)throttle,
                           (unsigned)((pad_active ? 1u : 0u) |
                                      (s_cmd_arm ? 2u : 0u) |
                                      (s_ramping ? 4u : 0u)),
                           (unsigned)s_fault, (unsigned)s_crc_errors, raw_adc,
                           brake_raw, (long)brake_cmd, brake_fault ? 1 : 0,
                           // 14th field, added 2026-09-08: the pad cruise
                           // setpoint in volts, so an outdoor throttle-to-speed
                           // run is recorded automatically instead of on paper.
                           // Parsers tolerate extra fields by design.
                           cruise_v,
                           // 15th field, added 2026-10-01. The ACTUAL DAC
                           // voltage on the wire. Item 17 asks whether the
                           // throttle sits in the measured 1.14-1.66 V band,
                           // and nothing in $T carried it -- field 6 is the PAD
                           // throttle, which reads 0 while Autoware drives.
                           // Parsers tolerate extra fields by design.
                           dac_v);
    if (n <= 0 || n >= (int)sizeof(body)) return;

    char out[JLINK_LINE_MAX + 8];
    const int m = snprintf(out, sizeof(out), "$%s*%02X\r\n", body,
                           nmea_xor(body, n));
    if (m > 0) uart_write_bytes(JLINK_UART_NUM, out, m);
}
