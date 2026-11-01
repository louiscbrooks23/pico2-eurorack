// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Louis Brooks

#pragma once
#include <stdint.h>
#include <stddef.h>
void multicore_launch_core1_with_stack(void (*entry)(void), uint32_t*, size_t);
