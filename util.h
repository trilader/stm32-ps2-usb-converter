#pragma once
#include <stdint.h>

#define UNUSED(x) ((void)x);
extern unsigned long systicks;

inline void delay(uint32_t usecs)
{
    uint32_t ctr=(48000000ul*usecs)/3/1000000;
    while(ctr)
    {
        __asm("");
        --ctr;
    }
}
