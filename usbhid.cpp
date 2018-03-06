/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/hid.h>
#include <libopencm3/stm32/exti.h>

#include "stdio.h"

#include "util.h"
#include "usb.h"
#include "serial.h"
#include "ps2handler.h"

extern usbd_device *usbd_dev;
unsigned long systicks=0;
ps2handler ps2keyboard;


bool keys_need_update=false;
bool f1=false, f2=false, f3=false, f4=false;

void update_leds(bool num, bool caps, bool scroll)
{
    ps2keyboard.update_leds(num,caps,scroll);
}

int main(void)
{
    // Disable JTAG, enable SWD. This frees up GPIO PA15, PB3 and PB4
    AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;

    rcc_clock_setup_in_hsi_out_48mhz();

    setup_serial();

    printf("\n\n\n");
    printf("Booting...\n");
    printf("Built on %s %s\n", __DATE__, __TIME__);

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    /*gpio_set_mode(GPIOA, GPIO_MODE_INPUT, 0, GPIO15);
    gpio_set(GPIOA, GPIO15);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, PIO_CNF_OUTPUT_PUSHPULL, GPIO15);*/

    printf("Configuring USB\n");
    setup_usb();

    // GPIO C13 is the onboard LED
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    // Disable the led. It is active LOW.
    gpio_set(GPIOC, GPIO13);


    // GPIO pins for media keys. Set to input and enable internal pulldown
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);
    gpio_clear(GPIOA, GPIO0);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO1);
    gpio_clear(GPIOA, GPIO1);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO2);
    gpio_clear(GPIOA, GPIO2);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO3);
    gpio_clear(GPIOA, GPIO3);

    printf("Configuring Interupts\n");

#define EXTI_GO(n,m) exti_enable_request(n); exti_set_trigger(n,m);

    EXTI_GO(EXTI0, EXTI_TRIGGER_BOTH);
    EXTI_GO(EXTI1, EXTI_TRIGGER_BOTH);
    EXTI_GO(EXTI2, EXTI_TRIGGER_BOTH);
    EXTI_GO(EXTI3, EXTI_TRIGGER_BOTH);
    //EXTI_GO(EXTI4, EXTI_TRIGGER_BOTH);

    EXTI_GO(EXTI7, EXTI_TRIGGER_BOTH);

    nvic_enable_irq(NVIC_EXTI0_IRQ);
    nvic_enable_irq(NVIC_EXTI1_IRQ);
    nvic_enable_irq(NVIC_EXTI2_IRQ);
    nvic_enable_irq(NVIC_EXTI3_IRQ);
    //nvic_enable_irq(NVIC_EXTI4_IRQ);

    nvic_enable_irq(NVIC_EXTI9_5_IRQ);
    //Make sure systick doesn't interrupt PS/2 protocol bitbang action
    nvic_set_priority(NVIC_SYSTICK_IRQ, 255);

    //nvic_enable_irq(NVIC_EXTI15_10_IRQ);

    printf("Configuring SysTick\n");
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
    /* SysTick interrupt every N clock pulses: set reload to N-1 */
    systick_set_reload((48000000/8/1000)-1);
    systick_interrupt_enable();
    systick_counter_enable();

    printf("Enabling PS/2 Port\n");
    ps2keyboard.init();

    while (1)
        //__asm("wfi");
        usbd_poll(usbd_dev);
}

bool read_gpio_debounced(int bank, int port)
{
    constexpr unsigned int _1ms=48000000/3/1000;
    constexpr unsigned int _500us=48000000/3/2000;

    UNUSED(_1ms);

    uint16_t try1=gpio_get(bank, port);

    for(unsigned int i=0; i<_500us; i++)
        __asm__("nop");

    uint16_t try2=gpio_get(bank, port);

    for(unsigned int i=0; i<_500us; i++)
        __asm__("nop");

    uint16_t try3=gpio_get(bank, port);

    uint8_t result=(!!try1) + (!!try2) + (!!try3);

    return result>=2;
}

void exti0_isr(void)
{
    if(exti_get_flag_status(EXTI0))
    {
        bool state=read_gpio_debounced(GPIOA, GPIO0);
        if(state!=f1)
        {
            f1=state;
            keys_need_update=true;
            printf("%08lu GPIO A0 changed state to %d\n",systicks, f1);
        }
        exti_reset_request(EXTI0);
    }
}

void exti1_isr(void)
{
    if(exti_get_flag_status(EXTI1))
    {
        bool state=read_gpio_debounced(GPIOA, GPIO1);
        if(state!=f2)
        {
            f2=state;
            keys_need_update=true;
            printf("%08lu GPIO A1 changed state to %d\n",systicks, f2);
        }
        exti_reset_request(EXTI1);
    }
}


void exti2_isr(void)
{
    if(exti_get_flag_status(EXTI2))
    {
        bool state=read_gpio_debounced(GPIOA, GPIO2);
        if(state!=f3)
        {
            f3=state;
            keys_need_update=true;
            printf("%08lu GPIO A2 changed state to %d\n",systicks, f3);
        }
        exti_reset_request(EXTI2);
    }
}

void exti3_isr(void)
{
    if(exti_get_flag_status(EXTI3))
    {
        bool state=read_gpio_debounced(GPIOA, GPIO3);
        if(state!=f4)
        {
            f4=state;
            keys_need_update=true;
            printf("%08lu GPIO A3 changed state to %d\n",systicks, f4);
        }
        exti_reset_request(EXTI3);
    }
}

void exti9_5_isr(void)
{
    if(exti_get_flag_status(EXTI7))
    {
        bool clock_state=gpio_get(GPIOA, GPIO7);
        bool data_state=gpio_get(GPIOA, GPIO6);

        ps2keyboard.clock_update(clock_state, data_state);
        exti_reset_request(EXTI7);
    }
}

/*
void exti15_10_isr(void)
{
    printf("%s\n",__PRETTY_FUNCTION__);
}
*/

void hard_fault_handler()
{
    printf("Hard Fault!");
    while(1)
    {
        ; // Do nothing
    }
}

void sys_tick_handler(void)
{
    static int ticks=0;
    systicks++;

    ticks++;
    if(ticks>500)
    {
        gpio_toggle(GPIOC, GPIO13);
        ticks=0;
    }

    static int last_ps2_fails=0;

    ps2keyboard.decode_scancode();

    if(ps2keyboard.fail_count!=last_ps2_fails)
    {
        printf("PS/2 failure count: %03d\n", ps2keyboard.fail_count);
        last_ps2_fails=ps2keyboard.fail_count;
    }

    if(ps2keyboard.need_update)
    {
        //ps2keyboard.need_update=false;
        uint8_t buf[8]={
            ps2keyboard.usb_modifier_byte(), // modifiers
            0, // reserverd
            0, // leds,
            ps2keyboard.usb_keys[0],
            ps2keyboard.usb_keys[1],
            ps2keyboard.usb_keys[2],
            ps2keyboard.usb_keys[3],
            ps2keyboard.usb_keys[4],
        };

        uint16_t sent_bytes=usbd_ep_write_packet(usbd_dev, 0x81, buf, 8);

        if(sent_bytes)
        {
            // After we've successfully sent the key state to the host we can clean up the key tracking table.
            ps2keyboard.need_update=false;
            for(size_t index=0; index<ps2keyboard.usb_keys.size();++index)
            {
                // Skip over unused slots
                if(ps2keyboard.usb_keys[index]==0)
                    continue;

                // If the key was presed before we sent the last package to the host, mark it as sent.
                if(ps2keyboard.usb_keys_state[index]&ps2keyboard.usb_key_state_is_down)
                {
                    ps2keyboard.usb_keys_state[index]|=ps2keyboard.usb_key_state_is_sent;
                }
                // If we've sent it before (state already was set to sent) we can clean up this slot (but we need to check again for changes the next time around)
                else
                {
                    ps2keyboard.usb_keys[index]=0;
                    ps2keyboard.usb_keys_state[index]=0;
                    ps2keyboard.need_update=true;
                }
            }
        }
    }

    if(keys_need_update)
    {
        volatile uint8_t keycode_media_next        = 0b00000001;
        volatile uint8_t keycode_media_prev        = 0b00000010;
        volatile uint8_t keycode_media_stop        = 0b00000100;
        volatile uint8_t keycode_media_playpause   = 0b00001000;
        volatile uint8_t keycode_media_mute        = 0b00010000;
        volatile uint8_t keycode_media_volume_up   = 0b00100000;
        volatile uint8_t keycode_media_volume_down = 0b01000000;

        UNUSED(keycode_media_stop)
        UNUSED(keycode_media_volume_up);
        UNUSED(keycode_media_volume_down);

        uint8_t buf2[1]={0};

        if(f1)
            buf2[0]|=keycode_media_prev;
        if(f2)
            buf2[0]|=keycode_media_next;
        if(f3)
            buf2[0]|=keycode_media_playpause;
        if(f4)
            buf2[0]|=keycode_media_mute;

        usbd_ep_write_packet(usbd_dev, 0x82, buf2, 1);

        //Clear key update required flag
        keys_need_update=false;
    }
}
