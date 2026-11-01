// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Louis Brooks

#pragma once
#include <stdint.h>
struct dma_hw_t { volatile uint32_t ints0; }; extern dma_hw_t* dma_hw;
struct dma_channel_config { int x; };
enum { DMA_SIZE_32 = 2 };
int dma_claim_unused_channel(bool); dma_channel_config dma_channel_get_default_config(uint);
void channel_config_set_transfer_data_size(dma_channel_config*,int); void channel_config_set_read_increment(dma_channel_config*,bool);
void channel_config_set_write_increment(dma_channel_config*,bool); void channel_config_set_dreq(dma_channel_config*,unsigned);
void channel_config_set_chain_to(dma_channel_config*,unsigned);
void dma_channel_configure(unsigned,const dma_channel_config*,volatile void*,const volatile void*,unsigned,bool);
void dma_channel_set_write_addr(unsigned,volatile void*,bool); void dma_channel_set_read_addr(unsigned,const volatile void*,bool);
void dma_channel_set_irq0_enabled(unsigned,bool); void dma_start_channel_mask(unsigned);
