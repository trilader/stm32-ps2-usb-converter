#pragma once

#include <libopencm3/usb/usbd.h>

#ifdef __cplusplus
extern "C" {
#endif

extern usbd_device *usbd_dev;
void setup_usb(void);

#ifdef __cplusplus
}
#endif
