// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Louis Brooks

#pragma once
#include <stdint.h>
struct pio_hw { volatile uint32_t txf[4]; volatile uint32_t rxf[4]; };
typedef pio_hw* PIO; extern PIO pio0;
struct pio_program { int x; };
unsigned pio_add_program(PIO, const pio_program*); unsigned pio_get_dreq(PIO,unsigned,bool);
void pio_enable_sm_mask_in_sync(PIO, unsigned);
