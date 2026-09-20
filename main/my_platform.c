// Example-derived Bluepad32 platform glue for Xbox controller to DAC mapping.

#include <string.h>

#include <uni.h>

#include "controller_dac.h"

typedef struct my_platform_instance_s {
    uni_gamepad_seat_t gamepad_seat;
} my_platform_instance_t;

static my_platform_instance_t* get_my_platform_instance(uni_hid_device_t* d);
static void trigger_event_on_gamepad(uni_hid_device_t* d);

// ---- LINK KEEP-ALIVE -----------------------------------------------------
// MEASURED 2026-09-08: this pad transmits ONLY ON CHANGE. Held still or centred
// it goes completely silent, and a silent Bluetooth link is eventually dropped
// by the controller -- observed repeatedly at roughly 10 s of idle. That is
// fatal for autonomy, because during an autonomous run the operator holds the
// pad DELIBERATELY STILL, which is exactly the condition that kills the link.
// The whole afternoon was worked around by tapping a button once a second.
//
// Sending an output report keeps real traffic on the link. set_player_leds is
// used rather than rumble: it is silent and the operator does not feel it.
//
// This MUST run on the BTstack thread, so it is a btstack run-loop timer rather
// than a call from the control loop on the other core.
// NOTE: this does NOT affect pad_active -- that is the Bluetooth link state
// now, not report timing. Kept only because periodic traffic is good for link
// health. Measured 2026-09-08: it did not prevent the drop, because the drop
// was a firmware staleness timeout, not a Bluetooth disconnect.
#define PAD_KEEPALIVE_MS 5000
static btstack_timer_source_t s_keepalive_timer;
static uni_hid_device_t* s_pad_device = NULL;

static void pad_keepalive_handler(btstack_timer_source_t* ts) {
    if (s_pad_device != NULL) {
        my_platform_instance_t* ins = get_my_platform_instance(s_pad_device);
        static int n = 0;
        const bool has_leds   = s_pad_device->report_parser.set_player_leds != NULL;
        const bool has_rumble = s_pad_device->report_parser.play_dual_rumble != NULL;
        if (has_leds) {
            s_pad_device->report_parser.set_player_leds(s_pad_device,
                                                        ins->gamepad_seat);
        } else if (has_rumble) {
            // Fallback: the shortest, weakest rumble the API allows. Some
            // parsers implement rumble but not LEDs.
            s_pad_device->report_parser.play_dual_rumble(s_pad_device, 0, 1, 1, 1);
        }
        if ((n++ % 5) == 0) {
            logi("custom: keepalive tick %d leds=%d rumble=%d\n",
                 n, (int)has_leds, (int)has_rumble);
        }
    }
    btstack_run_loop_set_timer(ts, PAD_KEEPALIVE_MS);
    btstack_run_loop_add_timer(ts);
}

static void my_platform_init(int argc, const char** argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    logi("custom: init()\n");
}

static void my_platform_on_init_complete(void) {
    logi("custom: on_init_complete()\n");

    uni_bt_start_scanning_and_autoconnect_unsafe();
    uni_bt_allow_incoming_connections(true);
}

static uni_error_t my_platform_on_device_discovered(bd_addr_t addr, const char* name, uint16_t cod, uint8_t rssi) {
    (void)addr;
    ARG_UNUSED(name);
    ARG_UNUSED(rssi);

    if (((cod & UNI_BT_COD_MINOR_MASK) & UNI_BT_COD_MINOR_KEYBOARD) == UNI_BT_COD_MINOR_KEYBOARD) {
        return UNI_ERROR_IGNORE_DEVICE;
    }

    return UNI_ERROR_SUCCESS;
}

static void my_platform_on_device_connected(uni_hid_device_t* d) {
    logi("custom: device connected: %p\n", d);
}

static void my_platform_on_device_disconnected(uni_hid_device_t* d) {
    logi("custom: device disconnected: %p\n", d);
    if (s_pad_device == d) {
        s_pad_device = NULL;
    }
    xbox_dac_on_controller_connected(false);
}

static uni_error_t my_platform_on_device_ready(uni_hid_device_t* d) {
    logi("custom: device ready: %p\n", d);

    my_platform_instance_t* ins = get_my_platform_instance(d);
    ins->gamepad_seat = GAMEPAD_SEAT_A;

    xbox_dac_on_controller_connected(true);
    trigger_event_on_gamepad(d);

    // Start (or re-target) the keep-alive now that a pad is actually present.
    s_pad_device = d;
    btstack_run_loop_remove_timer(&s_keepalive_timer);
    btstack_run_loop_set_timer_handler(&s_keepalive_timer, pad_keepalive_handler);
    btstack_run_loop_set_timer(&s_keepalive_timer, PAD_KEEPALIVE_MS);
    btstack_run_loop_add_timer(&s_keepalive_timer);
    logi("custom: pad keep-alive every %d ms\n", PAD_KEEPALIVE_MS);

    return UNI_ERROR_SUCCESS;
}

static void my_platform_on_controller_data(uni_hid_device_t* d, uni_controller_t* ctl) {
    static uni_controller_t prev = {0};

    ARG_UNUSED(d);

    // Stamp liveness BEFORE the duplicate filter below. The filter exists to
    // skip redundant downstream work, but a held stick sends identical reports
    // forever -- so timestamping after it made a motionless stick look like a
    // dead controller, and the staleness check then zeroed the steering duty
    // mid-stroke.
    xbox_dac_note_controller_alive();

    if (memcmp(&prev, ctl, sizeof(*ctl)) == 0) {
        return;
    }

    prev = *ctl;
    xbox_dac_on_controller_data(ctl);
}

static const uni_property_t* my_platform_get_property(uni_property_idx_t idx) {
    ARG_UNUSED(idx);
    return NULL;
}

static void my_platform_on_oob_event(uni_platform_oob_event_t event, void* data) {
    switch (event) {
        case UNI_PLATFORM_OOB_GAMEPAD_SYSTEM_BUTTON:
            if (data != NULL) {
                trigger_event_on_gamepad((uni_hid_device_t*)data);
            }
            break;

        case UNI_PLATFORM_OOB_BLUETOOTH_ENABLED:
            logi("custom: Bluetooth enabled: %d\n", (bool)(data));
            break;

        default:
            break;
    }
}

static my_platform_instance_t* get_my_platform_instance(uni_hid_device_t* d) {
    return (my_platform_instance_t*)&d->platform_data[0];
}

static void trigger_event_on_gamepad(uni_hid_device_t* d) {
    my_platform_instance_t* ins = get_my_platform_instance(d);

    if (d->report_parser.play_dual_rumble != NULL) {
        d->report_parser.play_dual_rumble(d, 0, 150, 128, 40);
    }

    if (d->report_parser.set_player_leds != NULL) {
        d->report_parser.set_player_leds(d, ins->gamepad_seat);
    }
}

struct uni_platform* get_my_platform(void) {
    static struct uni_platform plat = {
        .name = "custom",
        .init = my_platform_init,
        .on_init_complete = my_platform_on_init_complete,
        .on_device_discovered = my_platform_on_device_discovered,
        .on_device_connected = my_platform_on_device_connected,
        .on_device_disconnected = my_platform_on_device_disconnected,
        .on_device_ready = my_platform_on_device_ready,
        .on_oob_event = my_platform_on_oob_event,
        .on_controller_data = my_platform_on_controller_data,
        .get_property = my_platform_get_property,
    };

    return &plat;
}