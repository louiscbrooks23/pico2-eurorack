// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Louis Brooks

#pragma once
#include <stdint.h>
#include <stddef.h>
struct i2c_inst; extern i2c_inst *i2c0, *i2c1;
void i2c_init(i2c_inst*, unsigned); int i2c_write_blocking(i2c_inst*, uint8_t, const uint8_t*, size_t, bool);
