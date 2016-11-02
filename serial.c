#include "serial.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#include <errno.h>

// See:
// https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f1/stm32-h103/usart_irq_printf/usart_irq_printf.c

typedef int32_t ring_size_t;

struct ring
{
    uint8_t *data;
    ring_size_t size;
    uint32_t begin;
    uint32_t end;
};

#define RING_SIZE(RING)  ((RING)->size-1)
#define RING_DATA(RING)  (RING)->data
#define RING_EMPTY(RING) ((RING)->begin==(RING)->end)

static void ring_init(struct ring *ring, uint8_t *buf, ring_size_t size)
{
    ring->data = buf;
    ring->size = size;
    ring->begin = 0;
    ring->end = 0;
}

static int32_t ring_write_ch(struct ring *ring, uint8_t ch)
{
    if(((ring->end+1)%ring->size)!=ring->begin)
    {
        ring->data[ring->end++]=ch;
        ring->end%=ring->size;
        return (uint32_t)ch;
    }
    return -1;
}

static int32_t ring_write(struct ring *ring, uint8_t *data, ring_size_t size)
{
    int32_t i;
    for(i=0;i<size;i++)
    {
        if (ring_write_ch(ring,data[i])<0)
        {
            return -i;
        }
    }

    return i;
}

static int32_t ring_read_ch(struct ring *ring, uint8_t *ch)
{
    int32_t ret=-1;

    if(ring->begin!=ring->end)
    {
        ret=ring->data[ring->begin++];
        ring->begin%=ring->size;
        if (ch)
        {
            *ch = ret;
        }
    }

    return ret;
}

#define BUFFER_SIZE 1024

struct ring output_ring;
uint8_t output_ring_buffer[BUFFER_SIZE];

void setup_serial(void)
{
    /* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_USART1);

    /* Initialize output ring buffer. */
    ring_init(&output_ring, output_ring_buffer, BUFFER_SIZE);

    /* Enable the USART1 interrupt. */
    nvic_enable_irq(NVIC_USART1_IRQ);


    /* Setup GPIO pin GPIO_USART1_RE_TX on GPIO port B for transmit. */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

    /* Setup UART parameters. */
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX);

    /* Finally enable the USART. */
    usart_enable(USART1);
}

// Make the compiler shut up about missing prototypes
int _write(int,char*,int);

/*
//Blocking Version
int _write(int file, char *ptr, int len)
{
    int i;

    if (file == 1) {
        for (i = 0; i < len; i++)
            usart_send_blocking(USART1, ptr[i]);
        return i;
    }

    errno = EIO;
    return -1;
}*/

void usart1_isr(void)
{
    /* Check if we were called because of RXNE. */
//    if(((USART_CR1(USART1)&USART_CR1_RXNEIE)!=0) &&
//	     ((USART_SR(USART1)&USART_SR_RXNE)!=0))
//    {
//
//		/* Indicate that we got data. */
//		gpio_toggle(GPIOC, GPIO12);
//
//		/* Retrieve the data from the peripheral. */
//		ring_write_ch(&output_ring, usart_recv(USART1));
//
//		/* Enable transmit interrupt so it sends back the data. */
//		USART_CR1(USART1) |= USART_CR1_TXEIE;
//	}

    /* Check if we were called because of TXE. */
    if(((USART_CR1(USART1) & USART_CR1_TXEIE)!=0) &&
       ((USART_SR(USART1) & USART_SR_TXE)!=0)) {

        int32_t data;

        data=ring_read_ch(&output_ring, NULL);

        if (data == -1) {
            /* Disable the TXE interrupt, it's no longer needed. */
            USART_CR1(USART1) &= ~USART_CR1_TXEIE;
        } else {
            /* Put data into the transmit register. */
            usart_send(USART1, data);
        }
    }
}

int _write(int file, char *ptr, int len)
{
    int ret;

    if (file == 1) {
        ret = ring_write(&output_ring, (uint8_t *)ptr, len);

        if (ret < 0)
            ret = -ret;

        // Enable Transmit request
        USART_CR1(USART1) |= USART_CR1_TXEIE;

        return ret;
    }

    errno = EIO;
    return -1;
}
