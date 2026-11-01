// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Louis Brooks

#pragma once
#include <stdint.h>
#include <stddef.h>
struct spi_inst; extern spi_inst* spi0;
void spi_init(spi_inst*, unsigned); int spi_write_read_blocking(spi_inst*, const uint8_t*, uint8_t*, size_t);
