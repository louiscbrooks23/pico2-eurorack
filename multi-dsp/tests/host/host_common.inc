// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Louis Brooks

// Host-side stand-ins for the Pico SDK so main.cpp can be compiled and exercised on a PC.
// time_us_32() is a fake clock: it advances by `next_cost` microseconds on every second call,
// which is how tests simulate the cost of a chunk (see test_cpu_stats.cpp).
#include <cstdio>
#include <cmath>
#include "pico/stdlib.h"
static uint32_t fake_us = 0, fake_ms = 100000;
static int tcalls=0; static uint32_t next_cost=0; uint32_t time_us_32(){ if (tcalls++ & 1) fake_us += next_cost; return fake_us; } uint32_t to_ms_since_boot(absolute_time_t t){return fake_ms;} absolute_time_t get_absolute_time(){return 0;}
void sleep_ms(uint32_t){} void sleep_us(uint32_t){}
void gpio_init(uint){} void gpio_set_dir(uint,int){} void gpio_put(uint,int){} bool gpio_get(uint){return true;}
void gpio_pull_up(uint){} void gpio_set_function(uint,int){}
void gpio_set_irq_enabled_with_callback(uint,uint32_t,bool,gpio_irq_callback_t){} void gpio_set_irq_enabled(uint,uint32_t,bool){}
bool set_sys_clock_khz(uint32_t,bool){return true;} void stdio_init_all(){} void tight_loop_contents(){}
#include "pico/multicore.h"
void multicore_launch_core1_with_stack(void(*)(void),uint32_t*,size_t){}
#include "hardware/sync.h"
uint32_t save_and_disable_interrupts(){return 0;} void restore_interrupts(uint32_t){}
#include "hardware/irq.h"
void irq_set_exclusive_handler(unsigned,void(*)()){} void irq_set_enabled(unsigned,bool){} void irq_set_priority(unsigned,unsigned char){}
#include "hardware/i2c.h"
i2c_inst *i2c0,*i2c1; void i2c_init(i2c_inst*,unsigned){}
static int i2c_bytes = 0, i2c_calls = 0;
int i2c_write_blocking(i2c_inst*,uint8_t,const uint8_t*,size_t n,bool){ i2c_bytes += n; i2c_calls++; return 0;}
#include "hardware/spi.h"
spi_inst* spi0; void spi_init(spi_inst*,unsigned){} int spi_write_read_blocking(spi_inst*,const uint8_t*,uint8_t*,size_t){return 0;}
#include "hardware/pwm.h"
unsigned pwm_gpio_to_slice_num(unsigned){return 0;} unsigned pwm_gpio_to_channel(unsigned){return 0;}
void pwm_set_wrap(unsigned,unsigned short){} void pwm_set_chan_level(unsigned,unsigned,unsigned short){} void pwm_set_enabled(unsigned,bool){}
#include "hardware/adc.h"
void adc_init(){} void adc_gpio_init(unsigned){} void adc_select_input(unsigned){} uint16_t adc_read(){return 0;}
#include "i2s.pio.h"
PIO pio0; unsigned pio_add_program(PIO,const pio_program*){return 0;} unsigned pio_get_dreq(PIO,unsigned,bool){return 0;}
void pio_enable_sm_mask_in_sync(PIO,unsigned){}
const pio_program i2s_tx_program{0}, i2s_rx_program{0};
void i2s_tx_program_init(PIO,unsigned,unsigned,unsigned,unsigned){} void i2s_rx_program_init(PIO,unsigned,unsigned,unsigned,unsigned){}
#include "hardware/dma.h"
dma_hw_t* dma_hw; int dma_claim_unused_channel(bool){return 0;} dma_channel_config dma_channel_get_default_config(uint){return {};}
void channel_config_set_transfer_data_size(dma_channel_config*,int){} void channel_config_set_read_increment(dma_channel_config*,bool){}
void channel_config_set_write_increment(dma_channel_config*,bool){} void channel_config_set_dreq(dma_channel_config*,unsigned){}
void channel_config_set_chain_to(dma_channel_config*,unsigned){}
void dma_channel_configure(unsigned,const dma_channel_config*,volatile void*,const volatile void*,unsigned,bool){}
void dma_channel_set_write_addr(unsigned,volatile void*,bool){} void dma_channel_set_read_addr(unsigned,const volatile void*,bool){}
void dma_channel_set_irq0_enabled(unsigned,bool){} void dma_start_channel_mask(unsigned){}
