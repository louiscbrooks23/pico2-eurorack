// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Louis Brooks

#pragma once
#include <stdint.h>
void adc_init(); void adc_gpio_init(unsigned); void adc_select_input(unsigned); uint16_t adc_read();
