#include "usb.h"

#include "util.h"
#include <stdlib.h>
#include <stdio.h>
#include <libopencm3/stm32/tools.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>
#include <libopencm3/stm32/st_usbfs.h>


/* Define this to include the DFU APP interface. */
//#define INCLUDE_DFU_INTERFACE

#ifdef INCLUDE_DFU_INTERFACE
#include <libopencm3/cm3/scb.h>
#include <libopencm3/usb/dfu.h>
#endif

usbd_device *usbd_dev;

const struct usb_device_descriptor dev_descr = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x0483,
    .idProduct = 0x5710,
    .bcdDevice = 0x0200,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};


static const uint8_t hid_keyboard_report_descriptor[] = {
    0x05, 0x01,             // Usage Page (Generic Desktop),
    0x09, 0x06,             // Usage (Keyboard),
    0xA1, 0x01,             // Collection (Application),
    0x75, 0x01,             // Report Size (1),
    0x95, 0x08,             // Report Count (8),
    0x05, 0x07,             // Usage Page (Key Codes),
    0x19, 0xE0,             // Usage Minimum (224),
    0x29, 0xE7,             // Usage Maximum (231),
    0x15, 0x00,             // Logical Minimum (0),
    0x25, 0x01,             // Logical Maximum (1),
    0x81, 0x02,             // Input (Data, Variable, Absolute), ;Modifier byte
    //  byte 0: modifier
    0x95, 0x01,             // Report Count (1),
    0x75, 0x08,             // Report Size (8),
    0x81, 0x03,             // Input (Constant), ;Reserved byte
    // byte 1: reserved
    0x95, 0x05,             // Report Count (5),
    0x75, 0x01,             // Report Size (1),
    0x05, 0x08,             // Usage Page (LEDs),
    0x19, 0x01,             // Usage Minimum (1),
    0x29, 0x05,             // Usage Maximum (5),
    0x91, 0x02,             // Output (Data, Variable, Absolute), ;LED report
    0x95, 0x01,             // Report Count (1),
    0x75, 0x03,             // Report Size (3),
    0x91, 0x03,             // Output (Constant), ;LED report padding
    // byte 2: leds
    0x95, 0x06,             // Report Count (6),
    0x75, 0x08,             // Report Size (8),
    0x15, 0x00,             // Logical Minimum (0),
    0x25, 0x7F,             // Logical Maximum(104),
    0x05, 0x07,             // Usage Page (Key Codes),
    0x19, 0x00,             // Usage Minimum (0),
    0x29, 0x7F,             // Usage Maximum (104),
    0x81, 0x00,             // Input (Data, Array), ;Normal keys
    // bytes 3-9: keys
    0xc0                    // End Collection
};

static const uint8_t hid_mediakey_report_descriptor[] = {
    0x05, 0x0C,         // Usage Page (Consumer)
    0x09, 0x01,         // Usage (Consumer Controls)
    0xA1, 0x01,         // Collection (Application)

    0x15, 0x00,         // Logical Minimum (0)
    0x25, 0x01,         // Logical Maximum (1)
    0x75, 0x01,         // Report Size (1)

    0x95, 0x07,         // Report Count (5)
    0x09, 0xB5,         // Usage (Next Track)
    0x09, 0xB6,         // Usage (Previous Track)
    0x09, 0xB7,         // Usage (Stop)
    0x09, 0xCD,         // Usage (PlayPause)
    0x09, 0xE2,         // Usage (Mute)
    0x81, 0x06,         // Input (Data,Var,Rel)

    0x09, 0xE9,         // Usage (Volume +)
    0x09, 0xEA,         // Usage (Volume -)
    0x81, 0x02,         // Usage (Data,Var,Abs)
    // 7 bit keys
    0x95, 0x01,         // Report Count (1),
    0x81, 0x01,         // Input (Constant,Ary,Abs),
    // 1 bit padding
    0xC0                // End Collection
};

#if 0
static const uint8_t hid_report_descriptor[] = {
    0x05, 0x01, /* USAGE_PAGE (Generic Desktop)         */
    0x09, 0x02, /* USAGE (Mouse)                        */
    0xa1, 0x01, /* COLLECTION (Application)             */
    0x09, 0x01, /*   USAGE (Pointer)                    */
    0xa1, 0x00, /*   COLLECTION (Physical)              */
    0x05, 0x09, /*     USAGE_PAGE (Button)              */
    0x19, 0x01, /*     USAGE_MINIMUM (Button 1)         */
    0x29, 0x03, /*     USAGE_MAXIMUM (Button 3)         */
    0x15, 0x00, /*     LOGICAL_MINIMUM (0)              */
    0x25, 0x01, /*     LOGICAL_MAXIMUM (1)              */
    0x95, 0x03, /*     REPORT_COUNT (3)                 */
    0x75, 0x01, /*     REPORT_SIZE (1)                  */
    0x81, 0x02, /*     INPUT (Data,Var,Abs)             */
    0x95, 0x01, /*     REPORT_COUNT (1)                 */
    0x75, 0x05, /*     REPORT_SIZE (5)                  */
    0x81, 0x01, /*     INPUT (Cnst,Ary,Abs)             */
    0x05, 0x01, /*     USAGE_PAGE (Generic Desktop)     */
    0x09, 0x30, /*     USAGE (X)                        */
    0x09, 0x31, /*     USAGE (Y)                        */
    0x09, 0x38, /*     USAGE (Wheel)                    */
    0x15, 0x81, /*     LOGICAL_MINIMUM (-127)           */
    0x25, 0x7f, /*     LOGICAL_MAXIMUM (127)            */
    0x75, 0x08, /*     REPORT_SIZE (8)                  */
    0x95, 0x03, /*     REPORT_COUNT (3)                 */
    0x81, 0x06, /*     INPUT (Data,Var,Rel)             */
    0xc0,       /*   END_COLLECTION                     */
    0x09, 0x3c, /*   USAGE (Motion Wakeup)              */
    0x05, 0xff, /*   USAGE_PAGE (Vendor Defined Page 1) */
    0x09, 0x01, /*   USAGE (Vendor Usage 1)             */
    0x15, 0x00, /*   LOGICAL_MINIMUM (0)                */
    0x25, 0x01, /*   LOGICAL_MAXIMUM (1)                */
    0x75, 0x01, /*   REPORT_SIZE (1)                    */
    0x95, 0x02, /*   REPORT_COUNT (2)                   */
    0xb1, 0x22, /*   FEATURE (Data,Var,Abs,NPrf)        */
    0x75, 0x06, /*   REPORT_SIZE (6)                    */
    0x95, 0x01, /*   REPORT_COUNT (1)                   */
    0xb1, 0x01, /*   FEATURE (Cnst,Ary,Abs)             */
    0xc0        /* END_COLLECTION                       */
};
#endif

struct hid_function {
    struct usb_hid_descriptor hid_descriptor;
    struct {
        uint8_t bReportDescriptorType;
        uint16_t wDescriptorLength;
    } __attribute__((packed)) hid_report;
} __attribute__((packed));

static const struct hid_function keyboard_hid_function = {
    .hid_descriptor = {
        .bLength = sizeof(keyboard_hid_function),
        .bDescriptorType = USB_DT_HID,
        .bcdHID = 0x0100,
        .bCountryCode = 0,
        .bNumDescriptors = 1,
    },
    .hid_report = {
        .bReportDescriptorType = USB_DT_REPORT,
        .wDescriptorLength = sizeof(hid_keyboard_report_descriptor),
    }
};

static const struct hid_function mediakey_hid_function = {
    .hid_descriptor = {
        .bLength = sizeof(mediakey_hid_function),
        .bDescriptorType = USB_DT_HID,
        .bcdHID = 0x0100,
        .bCountryCode = 0,
        .bNumDescriptors = 1,
    },
    .hid_report = {
        .bReportDescriptorType = USB_DT_REPORT,
        .wDescriptorLength = sizeof(hid_mediakey_report_descriptor),
    }
};

const struct usb_endpoint_descriptor hid_keyboard_endpoint = {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x81,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 8,
    .bInterval = 0x20,
};

const struct usb_interface_descriptor hid_keyboard_iface = {
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,
    .bInterfaceClass = USB_CLASS_HID,
    .bInterfaceSubClass = 1, /* boot */
    .bInterfaceProtocol = 1, /* keyboard */
    .iInterface = 0,

    .endpoint = &hid_keyboard_endpoint,
    .extra = &keyboard_hid_function,
    .extralen = sizeof(keyboard_hid_function),
};


const struct usb_endpoint_descriptor hid_mediakey_endpoint = {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x82,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 8,
    .bInterval = 0x20,
};

const struct usb_interface_descriptor hid_mediakey_iface = {
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 1,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,
    .bInterfaceClass = USB_CLASS_HID,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,

    .endpoint = &hid_mediakey_endpoint,
    .extra = &mediakey_hid_function,
    .extralen = sizeof(mediakey_hid_function),
};


#ifdef INCLUDE_DFU_INTERFACE
const struct usb_dfu_descriptor dfu_function = {
    .bLength = sizeof(struct usb_dfu_descriptor),
    .bDescriptorType = DFU_FUNCTIONAL,
    .bmAttributes = USB_DFU_CAN_DOWNLOAD | USB_DFU_WILL_DETACH,
    .wDetachTimeout = 255,
    .wTransferSize = 1024,
    .bcdDFUVersion = 0x011A,
};

const struct usb_interface_descriptor dfu_iface = {
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 2,
    .bAlternateSetting = 0,
    .bNumEndpoints = 0,
    .bInterfaceClass = 0xFE,
    .bInterfaceSubClass = 1,
    .bInterfaceProtocol = 1,
    .iInterface = 0,

    .extra = &dfu_function,
    .extralen = sizeof(dfu_function),
};
#endif

const struct usb_interface ifaces[] = {
{
    .num_altsetting = 1,
    .altsetting = &hid_keyboard_iface,
},
{
    .num_altsetting = 1,
    .altsetting = &hid_mediakey_iface,
#ifdef INCLUDE_DFU_INTERFACE
}, {
    .num_altsetting = 1,
    .altsetting = &dfu_iface,
#endif
}};

const struct usb_config_descriptor config = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
#ifdef INCLUDE_DFU_INTERFACE
    .bNumInterfaces = 3,
#else
    .bNumInterfaces = 2,
#endif
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    // 7: Reserved (must be 1), 6: Self-Powered, 5: Remote-Wakeup, 4..0: Reserved (must be 0)
    .bmAttributes = 0b10100000,
    .bMaxPower = 0x32, // 2mA Units ->100 mA

    .interface = ifaces,
};

static const char *usb_strings[] = {
    "Black Sphere Technologies",
    "HID Demo",
    "DEMO",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static int hid_control_request_interface(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
            void (**complete)(usbd_device *, struct usb_setup_data *))
{
    UNUSED(complete);
    UNUSED(dev);

    if(req->bmRequestType==(USB_REQ_TYPE_CLASS|USB_REQ_TYPE_INTERFACE) /*0x21*/)
    {
        if((req->bRequest == USB_REQ_SET_CONFIGURATION) &&
           (req->wIndex==0 /* Must be 0 for SET_CONFIGURATION*/))
        {
            if(*len>0)
            {
                uint8_t data=**buf;

                // LEDs
                update_leds(data&0b001, data&0b010, data&0b100);

                return USBD_REQ_HANDLED;

            }
            return USBD_REQ_NOTSUPP;
        }
        else if(req->bRequest==0x0A /*SET_IDLE*/)
        {
            return USBD_REQ_HANDLED;
        }
    }

    printf("Got USB unsupported request: bmRequestType=%#x\nbRequest=%#x\nwValue=%#x\nwIndex=%#x\nwLenght=%#x\n\n",
           req->bmRequestType,
           req->bRequest,
           req->wValue,
           req->wIndex,
           req->wLength);

    return USBD_REQ_NOTSUPP;
}

static int hid_control_request(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
            void (**complete)(usbd_device *, struct usb_setup_data *))
{
    (void)complete;
    (void)dev;

    if((req->bmRequestType == (USB_REQ_TYPE_IN|USB_REQ_TYPE_STANDARD|USB_REQ_TYPE_INTERFACE) /*0x81*/) &&
       (req->bRequest == USB_REQ_GET_DESCRIPTOR /*0x06*/) &&
       (req->wValue == 0x2200 /*Descriptor Type (high byte) and Index (low byte)*/))
    {
        // req->wIndex is the interface number
        if(req->wIndex==0)
        {
            printf("Got request for keyboard report descriptor\n");
            *buf = (uint8_t *)hid_keyboard_report_descriptor;
            *len = sizeof(hid_keyboard_report_descriptor);
        }
        else if(req->wIndex==1)
        {
            printf("Got request for mediakey report descriptor\n");
            *buf = (uint8_t*)hid_mediakey_report_descriptor;
            *len = sizeof(hid_mediakey_report_descriptor);
        }
        else
        {
            return USBD_REQ_NOTSUPP;
        }
        return USBD_REQ_HANDLED;
    }

    printf("Got USB unsupported request: bmRequestType=%#x\nbRequest=%#x\nwValue=%#x\nwIndex=%#x\nwLenght=%#x\n\n",
           req->bmRequestType,
           req->bRequest,
           req->wValue,
           req->wIndex,
           req->wLength);

    return USBD_REQ_NOTSUPP;
}

#ifdef INCLUDE_DFU_INTERFACE
static void dfu_detach_complete(usbd_device *dev, struct usb_setup_data *req)
{
    (void)req;
    (void)dev;

    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, 0, GPIO15);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
              GPIO_CNF_OUTPUT_PUSHPULL, GPIO10);
    gpio_set(GPIOA, GPIO10);
    scb_reset_core();
}

static int dfu_control_request(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
            void (**complete)(usbd_device *, struct usb_setup_data *))
{
    (void)buf;
    (void)len;
    (void)dev;

    if ((req->bmRequestType != 0x21) || (req->bRequest != DFU_DETACH))
        return 0; /* Only accept class request. */

    *complete = dfu_detach_complete;

    return 1;
}
#endif

static void hid_set_config(usbd_device *dev, uint16_t wValue)
{
    (void)wValue;
    (void)dev;

    usbd_ep_setup(dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 8, NULL);
    usbd_ep_setup(dev, 0x82, USB_ENDPOINT_ATTR_INTERRUPT, 8, NULL);

    usbd_register_control_callback(
        dev,
        USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
        USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
        hid_control_request);

    usbd_register_control_callback(
        dev,
        USB_REQ_TYPE_CLASS|USB_REQ_TYPE_INTERFACE,
        USB_REQ_TYPE_DIRECTION|USB_REQ_TYPE_TYPE|USB_REQ_TYPE_RECIPIENT,
        hid_control_request_interface);

#ifdef INCLUDE_DFU_INTERFACE
    usbd_register_control_callback(
                dev,
                USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
                USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
                dfu_control_request);
#endif
}

static void handle_suspend(void)
{
    printf("%s called\n", __FUNCTION__);
    //update_leds(false, false, false); // Turn off leds when entering low power mode
    printf("USB_CNTR: %#08x\n", GET_REG(USB_CNTR_REG));
    SET_REG(USB_CNTR_REG, (GET_REG(USB_CNTR_REG)|USB_CNTR_FSUSP)); // Enter suspend mode
    //SET_REG(USB_CNTR_REG, (GET_REG(USB_CNTR_REG)|USB_CNTR_LP_MODE)); // Enter USB low power mode
    printf("USB_CNTR: %#08x\n", GET_REG(USB_CNTR_REG));
    printf("Done with handling USB suspend.\n");
}

static void handle_resume(void)
{
    printf("%s called\n", __FUNCTION__);
    printf("USB_CNTR: %#08x\n", GET_REG(USB_CNTR_REG));
    SET_REG(USB_CNTR_REG, (GET_REG(USB_CNTR_REG)&~USB_CNTR_FSUSP)); // Exit suspend mode
    printf("USB_CNTR: %#08x\n", GET_REG(USB_CNTR_REG));
    printf("Done with USB handling resume.\n");
}

static void handle_reset(void)
{
    printf("%s called\n", __FUNCTION__);
}

void setup_usb()
{
    usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
    usbd_register_set_config_callback(usbd_dev, hid_set_config);
    //usbd_register_suspend_callback(usbd_dev, &handle_suspend);
    //usbd_register_resume_callback(usbd_dev, &handle_resume);
    //usbd_register_reset_callback(usbd_dev, &handle_reset);
}
