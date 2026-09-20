// Jetson <-> ESP32 command link.
//
// Autoware sends high-level steering intent over a serial line; the ESP32
// remains the sole owner of every actuator and arbitrates between the Xbox pad
// (MANUAL) and the Jetson (AUTO).
//
// SCOPE: steering only. This module never touches the throttle DAC. In AUTO the
// human keeps the throttle trigger exactly as in MANUAL -- autonomy steers, the
// operator controls speed. That is deliberate, not an omission.

#pragma once

#include <stdbool.h>
#include <stdint.h>

// ---------------------------------------------------------------------------
// WHICH UART
//
// 0 = share the console UART0 on GPIO1/GPIO3, i.e. the CP2102 USB cable that is
//     already plugged in. Needs no wiring, so it is the default for bring-up.
//     Costs: ESP_LOG text interleaves with telemetry on TX (the Jetson filters
//     on the '$' prefix), and opening the port from the host reboots the ESP32,
//     so the host must open once and hold the handle.
//
// 2 = dedicated UART2 remapped to GPIO25/GPIO26. Three wires to the Jetson's
//     40-pin header (pin 8 TXD, pin 10 RXD, pin 6 GND -> /dev/ttyTHS1). No
//     reset-on-open, no log interleaving. This is the production answer.
//     UART1's default pins collide with SPI flash and UART2's collide with the
//     TWAI transceiver on GPIO16/17, so remapping is mandatory either way.
// ---------------------------------------------------------------------------
#define JLINK_UART_NUM 0
#define JLINK_TX_GPIO 25
#define JLINK_RX_GPIO 26
#define JLINK_BAUD 115200

#define JLINK_SHARES_CONSOLE (JLINK_UART_NUM == 0)

// $C steer field interpretation.
#define JLINK_STEER_MODE_DUTY  0   // steer is a duty, -0.95..0.95 (v1 behaviour)
#define JLINK_STEER_MODE_ANGLE 1   // steer is a TIRE ANGLE in radians (v2)

// flags bits in $C
#define JLINK_FLAG_ARM            0x01
#define JLINK_FLAG_AUTO           0x02
#define JLINK_FLAG_BRAKE_AUTH     0x04   // Jetson may drive the brake
#define JLINK_FLAG_THROTTLE_AUTH  0x08   // Jetson may drive the throttle

typedef enum {
    JLINK_MANUAL = 0,   // Xbox pad drives steering. Power-on default.
    JLINK_AUTO   = 1,   // Jetson drives steering. Pad still drives throttle.
    JLINK_FAULT  = 2,   // Latched. Steering duty forced to 0 until cleared.
} jlink_mode_t;

typedef enum {
    JLINK_FAULT_NONE = 0,
    JLINK_FAULT_TIMEOUT,      // no valid command within JLINK_CMD_TIMEOUT_MS
    JLINK_FAULT_CHECKSUM,     // too many bad frames in one second
    JLINK_FAULT_PAD_LOST,     // pad disconnected while in AUTO
    JLINK_FAULT_ADC,          // steering angle sensor read failed
    JLINK_FAULT_CAN,          // TWAI bus-off
    JLINK_FAULT_BRAKE,        // brake pot implausible
} jlink_fault_t;

// Start the link. Safe to call before or after the CAN/ADC init.
void jetson_link_start(void);

// Called from dac_task once per cycle.
//
//   manual_duty  what the pad is asking for, already limited and clamped
//   pad_active   pad connected AND its data is fresh
//   pad_centred  |axis| within the deadband
//   angle_deg    D34 shaft angle, or NAN if the read failed
//
// Returns the steering duty that should actually be sent to the SPARK MAX.
// Returns the steering duty to apply. In AUTO with steer mode ANGLE it closes
// a PI loop on the measured shaft angle; in mode DUTY it passes the command
// through as before.
float jetson_link_arbitrate(float manual_duty, bool pad_active, bool pad_centred,
                            float angle_deg);

// Longitudinal authority. Each returns true and writes *out only when the
// Jetson currently holds authority for that actuator AND the link is engaged;
// otherwise returns false and the pad value stands.
bool jetson_link_brake_override(float* out_0_1);
bool jetson_link_throttle_override(float* out_0_1);

// Publish one telemetry line. Called at JLINK_TELEM_HZ from dac_task.
void jetson_link_publish(float angle_deg, float applied_duty, int32_t axis,
                         int32_t throttle, bool pad_active, int raw_adc,
                         int brake_raw, int32_t brake_cmd, bool brake_fault,
                         float cruise_v);

jlink_mode_t jetson_link_mode(void);
void jetson_link_note_can_fault(void);
