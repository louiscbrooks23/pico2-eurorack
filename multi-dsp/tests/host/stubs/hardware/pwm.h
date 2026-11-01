// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Louis Brooks

#pragma once
unsigned pwm_gpio_to_slice_num(unsigned); unsigned pwm_gpio_to_channel(unsigned);
void pwm_set_wrap(unsigned,unsigned short); void pwm_set_chan_level(unsigned,unsigned,unsigned short); void pwm_set_enabled(unsigned,bool);
