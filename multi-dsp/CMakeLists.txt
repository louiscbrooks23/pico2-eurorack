# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Louis Brooks

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)
project(dev_dsp_module C CXX ASM)
pico_sdk_init()

add_executable(dev_dsp
    main.cpp
)


# Compile the PIO assembly into a C++ header
pico_generate_pio_header(dev_dsp ${CMAKE_CURRENT_LIST_DIR}/i2s.pio)

# Link essential hardware libraries
target_link_libraries(dev_dsp
    pico_stdlib
    pico_multicore
    hardware_i2c
    hardware_spi
    hardware_pio
    hardware_dma
    hardware_irq
    hardware_pwm
    hardware_clocks
    hardware_adc
    hardware_flash
    hardware_sync
)

pico_enable_stdio_usb(dev_dsp 1)
pico_enable_stdio_uart(dev_dsp 0)
pico_add_extra_outputs(dev_dsp)
