#pragma once

#include <stdbool.h>

#include <uni.h>

void xbox_dac_on_controller_connected(bool connected);
void xbox_dac_on_controller_data(const uni_controller_t* ctl);