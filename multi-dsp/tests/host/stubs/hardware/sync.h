// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Louis Brooks

#pragma once
#include <stdint.h>
uint32_t save_and_disable_interrupts(); void restore_interrupts(uint32_t);
