#pragma once

#include <stdbool.h>

#include <uni.h>

void xbox_dac_on_controller_connected(bool connected);
void xbox_dac_on_controller_data(const uni_controller_t* ctl);

// Called for EVERY controller report, including ones the platform layer drops
// as duplicates. Liveness must be judged on report arrival, not on the value
// changing: a stick held perfectly still produces identical reports, and
// treating those as silence makes the pad look disconnected.
void xbox_dac_note_controller_alive(void);