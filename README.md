# STM32: PS/2 to USB Converter

This code facilitates a STM32 chip to convert between PS/2 and USB. It is meant to connect a keyboard, which only provides a PS/2 connector, to a computer via USB.
This code powers on a daily basis:

- Cherry G80-3000LSMDE

## Dependencies

- `arm-none-eabi-gcc`
- `arm-none-eabi-gdb`
- `arm-none-eabi-binutils`
- `arm-none-eabi-newlib`
- `stlink`
- `openocd`
- `mpfr`

## Preparations

After cloning the repository you need to make the following preparations:

```
git submodule init
git submodule update
cd libopencm3
make
cd ..
make
```

## Hardware and Setup

You will obviously need a STM32 chip. At least, it should have connectors on PINs A6 and A7, as well on 5V and GND.
Take a freely connectable PS/2 socket and connect its 5V and GND with the STM. Also connect its DATA pin to A6 and its CLK pin to A7.
Add two pull-up resistors (10k Ohm works well) between A6/A7 and 5V, respectively.

Connect your STM via a microUSB/USB cable to your computer to power it up.
Use a ST-Link v2 Programmer (or similar) to flash the program using `make flash` onto the STM32.
You should then be able to connect your keyboard to the PS/2 socket and have it enumerated as an USB device on your computer.
