#pragma once

#include <stdlib.h>
#include <stdint.h>
#include <libopencm3/stm32/gpio.h>

class ps2handler
{
private:
    static constexpr uint8_t recv_buffer_size=64;
    volatile uint8_t recv_buffer[recv_buffer_size];
    volatile uint8_t recv_buffer_head=0, recv_buffer_tail=0;

    void init_recv_buffer();
    uint8_t get_scan_code();

public:
    ps2handler();
    void init();
    void clock_update(bool clock_state, bool data_state);
    void decode_scancode();
    const uint32_t data_pin_bank = GPIOA;
    const uint16_t data_pin_id = GPIO6;
    const uint32_t clock_pin_bank = GPIOA;
    const uint16_t clock_pin_id = GPIO7;

    bool fail_count=0;

};
