#pragma once

#include <stdlib.h>
#include <stdint.h>
#include <array>
#include <libopencm3/stm32/gpio.h>

class ps2handler
{
private:

    enum class protocol_state
    {
        receive,
        send_command,
        send_led_state,
        wait_for_response,
    };

    static constexpr uint8_t recv_buffer_size=64;
    volatile uint8_t recv_buffer[recv_buffer_size];
    volatile uint8_t recv_buffer_head=0, recv_buffer_tail=0;

    void init_recv_buffer();
    uint8_t get_scan_code();
    uint16_t map_to_state_index(uint8_t meta, uint8_t scancode) const;
    uint8_t map_state_to_usb_scancode(uint16_t state_index) const;

    const uint8_t meta_modifier=0x01;
    const uint8_t meta_pause_break=0x02;
    const uint8_t meta_break=0x04;

    protocol_state state;
    bool need_to_send=false;
    uint8_t led_byte=0;
    uint8_t current_send_bit=0;

    void clock_update_send(bool clock_state, bool data_state);
    void clock_update_receive(bool data_state);

    void send_start_bit(protocol_state next_state);

public:
    ps2handler();
    void init();
    void clock_update(bool clock_state, bool data_state);
    void decode_scancode();
    const uint32_t data_pin_bank = GPIOA;
    const uint16_t data_pin_id = GPIO6;
    const uint32_t clock_pin_bank = GPIOA;
    const uint16_t clock_pin_id = GPIO7;

    const uint8_t usb_key_state_is_sent = 0x01;
    const uint8_t usb_key_state_is_down = 0x02;

    std::array<uint8_t,5> usb_keys_state = {}; // State of the keys to be sent to the host. States: Empty (0), Down but unsent, Sent
    std::array<uint8_t,5> usb_keys = {}; // List of keys to be sent to the host. States: Empty (0), Scancode (not 0).
    std::array<uint8_t,1024> key_states = {};
    uint8_t usb_modifier_byte() const;
    void update_leds(bool num, bool caps, bool scroll);

    int fail_count=0;
    bool need_update=false;

};
