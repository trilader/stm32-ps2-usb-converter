#include "ps2handler.h"
#include "util.h"
#include <stdio.h>

int dbg_count=0;

template<typename... Args>
void debug_print(Args... args)
{
    /*dbg_count++;
    printf("dbg: %03d ",dbg_count);
    printf(args...);*/
}

// Initialize receive ringbuffer
void ps2handler::init_recv_buffer()
{
    for(uint8_t i=0; i<recv_buffer_size; ++i)
    {
        recv_buffer[i]=0;
    }
    recv_buffer_head=0;
    recv_buffer_tail=0;
}

// Fetches the next scancode from the receive ring buffer
uint8_t ps2handler::get_scan_code()
{
    uint8_t result,i;

    i=recv_buffer_tail;
    if(i==recv_buffer_head)
        return 0;
    i++;

    if(i>=recv_buffer_size)
        i=0;

    result=recv_buffer[i];
    recv_buffer_tail=i;
    return result;
}

static const uint8_t state_index_to_usb_scancode[]=
{
    0x00, 0x42, 0x00, 0x3e, 0x3c, 0x3a, 0x3b, 0x45,  // 0x0
    0x00, 0x43, 0x41, 0x3f, 0x3d, 0x2b, 0x35, 0x00,  // 0x8
    0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x1e, 0x00,  // 0x10
    0x00, 0x00, 0x1d, 0x16, 0x04, 0x1a, 0x1f, 0x00,  // 0x18
    0x00, 0x06, 0x1b, 0x07, 0x08, 0x21, 0x20, 0x00,  // 0x20
    0x00, 0x2c, 0x19, 0x09, 0x17, 0x15, 0x22, 0x00,  // 0x28
    0x00, 0x11, 0x05, 0x0b, 0x0a, 0x1c, 0x23, 0x00,  // 0x30
    0x00, 0x00, 0x10, 0x0d, 0x18, 0x24, 0x25, 0x00,  // 0x38
    0x00, 0x36, 0x0e, 0x0c, 0x12, 0x27, 0x26, 0x00,  // 0x40
    0x00, 0x37, 0x38, 0x0f, 0x33, 0x13, 0x2d, 0x00,  // 0x48
    0x00, 0x00, 0x34, 0x00, 0x2f, 0x2e, 0x00, 0x00,  // 0x50
    0x39, 0x00, 0x28, 0x30, 0x00, 0x31, 0x00, 0x00,  // 0x58
    0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x2a, 0x00,  // 0x60
    0x00, 0x59, 0x00, 0x5c, 0x5f, 0x00, 0x00, 0x00,  // 0x68
    0x62, 0x63, 0x5a, 0x5d, 0x5e, 0x60, 0x29, 0x53,  // 0x70
    0x44, 0x57, 0x5b, 0x56, 0x55, 0x61, 0x47, 0x00,  // 0x78
    0x00, 0x00, 0x00, 0x40, 0x46, 0x00, 0x00, 0x00,  // 0x80
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x88
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x90
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x98
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0xa0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0xa8
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0xb0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0xb8
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0xc0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0xc8
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0xd0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0xd8
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0xe0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0xe8
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0xf0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0xf8
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x100
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x108
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x110
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x118
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x120
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x65,  // 0x128
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x130
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x138
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x140
    0x00, 0x00, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x148
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x150
    0x00, 0x00, 0x58, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x158
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x160
    0x00, 0x4d, 0x00, 0x50, 0x4a, 0x00, 0x00, 0x00,  // 0x168
    0x49, 0x4c, 0x51, 0x00, 0x4f, 0x52, 0x00, 0x00,  // 0x170
    0x00, 0x00, 0x4e, 0x00, 0x46, 0x4b, 0x00, 0x00,  // 0x178
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x180
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x188
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x190
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x198
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x1a0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x1a8
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x1b0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x1b8
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x1c0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x1c8
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x1d0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x1d8
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x1e0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x1e8
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x1f0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x1f8
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x200
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // 0x208
    0x00, 0x00, 0x00, 0x00, 0x48, 0x00, 0x00, 0x00,  // 0x210
};

// Convert key event meta data and scancode into usb scancode translation LUT
// TODO: 2 bytes meta + 8 bytes scancode = 10 bits. Max index therefore is 0x3ff but the scancode translation table only goes up to 0x218. Why?
uint16_t ps2handler::map_to_state_index(uint8_t meta, uint8_t scancode) const
{
    return ((meta&0b11)<<8) + scancode;
}

uint8_t ps2handler::map_state_to_usb_scancode(uint16_t state_index) const
{
    return state_index_to_usb_scancode[state_index];
}

ps2handler::ps2handler(): state(ps2handler::protocol_state::receive)
{
    init_recv_buffer();
}

void ps2handler::init()
{
    gpio_set_mode(clock_pin_bank, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, clock_pin_id);
    gpio_set(clock_pin_bank, clock_pin_id);
    gpio_set_mode(data_pin_bank, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, data_pin_id);
    gpio_set(data_pin_bank, data_pin_id);

    //gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO10);
    //gpio_clear(GPIOB, GPIO10);
    //gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO11);
    //gpio_clear(GPIOB, GPIO11);
}

void ps2handler::clock_update_send(bool clock_state, bool data_state)
{
    uint8_t data_byte=(state==protocol_state::send_command)?0xED:led_byte;
    if(current_send_bit>=1 && current_send_bit<9)
    {
        if(clock_state==false)
        {
            //gpio_toggle(GPIOB, GPIO10);
            bool bit=data_byte&(1<<(current_send_bit-1));
            if(bit)
            {
                gpio_set(data_pin_bank, data_pin_id);
            }
            else
            {
                gpio_clear(data_pin_bank, data_pin_id);
            }
            //debug_print("sent bit %d\n",current_send_bit);
            current_send_bit++;
        }
    }
    else if(current_send_bit==9)
    {
        if(clock_state==false)
        {
            bool parity=!__builtin_parity(data_byte);
            if(parity)
            {
                gpio_set(data_pin_bank, data_pin_id);
            }
            else
                gpio_clear(data_pin_bank, data_pin_id);
            {
            }
            //debug_print("sent parity\n");
            current_send_bit++;
        }
    }
    else if(current_send_bit==10)
    {
        if(clock_state==false)
        {
            gpio_set(data_pin_bank, data_pin_id);
            current_send_bit++;
            //debug_print("sent stop\n");
        }
    }
    else if(current_send_bit==11)
    {
        if(clock_state==false)
        {
            if(data_state==false) // ACK bit ok
            {
                ; // Ignore ACK
            }
            else
            {
                printf("Error sending 0x%x to keyboard. NACK\n", data_byte);
                state=protocol_state::receive;

            }
            current_send_bit=0;

            if(state==protocol_state::send_led_state)
            {
                state=protocol_state::receive;
            }
            else
            {
                state=protocol_state::wait_for_response;
            }
        }
    }
}


void ps2handler::clock_update(bool clock_state, bool data_state)
{
    if(state==protocol_state::send_command||state==protocol_state::send_led_state)
    {
        clock_update_send(clock_state, data_state);
    }
    else
    {
        if(clock_state==false)
        {
            clock_update_receive(data_state);
        }
    }
}

void ps2handler::clock_update_receive(bool data_state)
{
    static uint8_t bit_index=0;

    static uint8_t start_bit=0xff;
    static uint8_t data_word=0;
    static uint8_t stop_bit=0xff;
    static uint8_t parity_bit=0xff;

    static uint32_t prev_ms=0;
    uint32_t now_ms;

    now_ms = systicks;
    if(now_ms-prev_ms>250)
    {
        bit_index=0;
        start_bit=0xff;
        data_word=0;
        stop_bit=0xff;
        parity_bit=0xff;
    }
    prev_ms=now_ms;

    /*gpio_toggle(GPIOB, GPIO10);
    if(data_state)
        gpio_set(GPIOB, GPIO11);
    else
        gpio_clear(GPIOB, GPIO11);
        */

    if(bit_index==0)
    {
        start_bit=data_state;
        if(start_bit!=0)
        {
            fail_count++;
            return;
        }
    }
    else if(bit_index>0 && bit_index<=8) // collect bits
    {
        data_word|=(data_state<<(bit_index-1));
    }
    else if(bit_index==9)
    {
        parity_bit=data_state;
    }
    else if(bit_index==10)
    {
        stop_bit=data_state;
    }
    bit_index++;
    if(bit_index==11) // 8 + start + stop + parity
    {
        bool parity_ok=__builtin_parity((data_word<<1)|parity_bit);
        if(start_bit==0 && stop_bit==1 && parity_ok)
        {
            if(state==protocol_state::receive)
            {
                uint8_t i=recv_buffer_head+1;
                if(i>=recv_buffer_size)
                    i=0;
                if(i!=recv_buffer_tail)
                {
                    recv_buffer[i]=data_word;
                    recv_buffer_head=i;
                }
            }
            else if(state==protocol_state::wait_for_response)
            {
                if(data_word==0xfa)
                {
                    send_start_bit(protocol_state::send_led_state);
                }
                else if(data_word==0xfe)
                {
                    send_start_bit(protocol_state::send_command);
                }
                else
                {
                    printf("Got unexpected command response: 0x%x\n", data_word);
                }
            }
        }
        else
        {
            fail_count++;
        }
        bit_index=0;
        data_word=0;
    }
}

void ps2handler::send_start_bit(protocol_state next_state)
{
    current_send_bit=0;
    state=next_state;
    gpio_set(data_pin_bank, data_pin_id);
    gpio_clear(clock_pin_bank, clock_pin_id);
    delay(30/*us*/);
    gpio_clear(data_pin_bank, data_pin_id);
    delay(30/*us*/);
    gpio_set(clock_pin_bank, clock_pin_id);
    current_send_bit=1;
}

void ps2handler::decode_scancode()
{
    uint8_t scancode;
    static uint8_t meta=0;
    static uint16_t prev_state_index=0;
    while((scancode=get_scan_code())!=0)
    {
        if(scancode==0xe0)
        {
            meta|=meta_modifier;
            continue;
        }
        if(scancode==0xe1)
        {
            meta|=meta_pause_break;
            continue;
        }
        if(scancode==0xf0)
        {
            meta|=meta_break;
            continue;
        }

        const uint16_t state_index=map_to_state_index(meta, scancode);
        if(prev_state_index==0x214 && state_index==0x77)
        {
            // This is needed to fix the behavior of the Pause/Break key
            // It sends an extra numlock state index
            meta=0;
            prev_state_index=0;
            continue;
        }
        prev_state_index=state_index;
        const bool last_key_state=key_states[state_index];
        const bool is_break=meta&meta_break;

        //debug_print("%#1x %#02x -> %#04x,\n", meta, scancode, state_index);

        if(last_key_state != !is_break)
        {
            need_update=true;
            uint8_t usb_scancode=map_state_to_usb_scancode(state_index);

            if(is_break)
            {
                if(usb_scancode!=0)
                {
                    // This key sends something on the usb side. Try to find the slot we've reserved for it when it was pressed.
                    for(size_t index=0;index<usb_keys.size();++index)
                    {
                        if(usb_keys[index]==usb_scancode)
                        {
                            // If the key was sent to the host we can free the slot is is occupying in the tracking table.
                            if(usb_keys_state[index]&usb_key_state_is_sent)
                            {
                                usb_keys[index]=0;
                            }
                            // Also reset the down/sent tracking table slot.
                            usb_keys_state[index]=0;
                            break;
                        }
                    }
                }
                // Remove key from internal state tracking table
                key_states[state_index]=false;
            }
            else
            {
                if(usb_scancode!=0)
                {
                    // This key sends something on the usb side. Try to find a free slot in the to send map.
                    for(size_t index=0;index<usb_keys.size();++index)
                    {
                        if(usb_keys[index]==0)
                        {
                            // We've found an empty slot. Put it in there and mark it as down in the state tracking table.
                            usb_keys_state[index]=usb_key_state_is_down;
                            usb_keys[index]=usb_scancode;
                            break;
                        }
                    }
                }
                // Mark key in internal state tracking table
                key_states[state_index]=true;
            }
        }
        meta=0;
    }
}

uint8_t ps2handler::usb_modifier_byte() const
{
    //LCtrl, LShift, LAlt, LGui, RCtrl, RShift, RAlt, RGui
    return (key_states[0x0014]<<0)
           + (key_states[0x0012]<<1)
           + (key_states[0x0011]<<2)
           + (key_states[0x011f]<<3)
           + (key_states[0x0114]<<4)
           + (key_states[0x0059]<<5)
           + (key_states[0x0111]<<6)
           + (key_states[0x0127]<<7);
}



void ps2handler::update_leds(bool num, bool caps, bool scroll)
{
    //printf("update_leds %d %d %d\n",num,caps,scroll);

    led_byte=(scroll<<0)|(num<<1)|(caps<<2);

    send_start_bit(protocol_state::send_command);
}

