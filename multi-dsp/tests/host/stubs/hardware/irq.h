// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Louis Brooks

#pragma once
#define DMA_IRQ_0 11
void irq_set_exclusive_handler(unsigned, void(*)()); void irq_set_enabled(unsigned,bool); void irq_set_priority(unsigned, unsigned char);
