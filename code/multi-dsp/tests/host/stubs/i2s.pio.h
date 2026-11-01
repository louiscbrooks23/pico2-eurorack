// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Louis Brooks

#pragma once
#include "hardware/pio.h"
extern const pio_program i2s_tx_program, i2s_rx_program;
void i2s_tx_program_init(PIO,unsigned,unsigned,unsigned,unsigned); void i2s_rx_program_init(PIO,unsigned,unsigned,unsigned,unsigned);
