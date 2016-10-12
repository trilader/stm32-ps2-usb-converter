#include "ps2handler.h"
#include "util.h"
#include <stdio.h>

void ps2handler::init_recv_buffer()
{
    for(uint8_t i=0; i<recv_buffer_size; ++i)
    {
        recv_buffer[i]=0;
    }
    recv_buffer_head=0;
    recv_buffer_tail=0;
}

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

ps2handler::ps2handler()
{
    init_recv_buffer();
}

void ps2handler::init()
{
    // Data pin
    gpio_set_mode(data_pin_bank, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, data_pin_id);
    // Enable pullup
    gpio_set(data_pin_bank, data_pin_id);

    // Clock pin
    gpio_set_mode(clock_pin_bank, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, clock_pin_id);
    // Enable pullup
    gpio_set(clock_pin_bank, clock_pin_id);

    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO10);
    gpio_clear(GPIOB, GPIO10);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO11);
    gpio_clear(GPIOB, GPIO11);
}

void ps2handler::clock_update(bool clock_state, bool data_state)
{
    UNUSED(clock_state);

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

    gpio_toggle(GPIOB, GPIO10);
    if(data_state)
        gpio_set(GPIOB, GPIO11);
    else
        gpio_clear(GPIOB, GPIO11);

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
            uint8_t i=recv_buffer_head+1;
            if(i>=recv_buffer_size)
                i=0;
            if(i!=recv_buffer_tail)
            {
                recv_buffer[i]=data_word;
                recv_buffer_head=i;
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

void ps2handler::decode_scancode()
{
    const uint8_t meta_break=0x01;
    const uint8_t meta_modifier=0x02;
    const uint8_t meta_pause_break=0x04;
    uint8_t scancode;
    static uint8_t meta=0;
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
        if(scancode&0x80)
        {
            meta|=meta_break;
            continue;
        }

        if(meta&meta_break)
        {
            printf("Release ");
            meta=(meta & ~meta_break);
        }
        else
        {
            printf("Press ");
        }

        if(meta&meta_modifier)
        {
            printf("E0 ");
            meta=(meta & ~meta_modifier);
        }
        printf("%#02x\n", scancode);
    }
}

