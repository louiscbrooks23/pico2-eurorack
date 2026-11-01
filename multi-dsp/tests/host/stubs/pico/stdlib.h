// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Louis Brooks

#pragma once
#include <stdint.h>
#include <stddef.h>
typedef unsigned int uint;
#define __isr
#define __not_in_flash_func(x) x
#define GPIO_IN 0
#define GPIO_OUT 1
#define GPIO_FUNC_I2C 3
#define GPIO_FUNC_SPI 1
#define GPIO_FUNC_PWM 4
#define GPIO_IRQ_EDGE_FALL 4
#define GPIO_IRQ_EDGE_RISE 8
typedef uint32_t absolute_time_t;
typedef void (*gpio_irq_callback_t)(uint, uint32_t);
uint32_t time_us_32();
uint32_t to_ms_since_boot(absolute_time_t);
absolute_time_t get_absolute_time();
void sleep_ms(uint32_t); void sleep_us(uint32_t);
void gpio_init(uint); void gpio_set_dir(uint,int); void gpio_put(uint,int); bool gpio_get(uint);
void gpio_pull_up(uint); void gpio_set_function(uint,int);
void gpio_set_irq_enabled_with_callback(uint, uint32_t, bool, gpio_irq_callback_t); void gpio_set_irq_enabled(uint, uint32_t, bool);
bool set_sys_clock_khz(uint32_t,bool); void stdio_init_all(); void tight_loop_contents();
