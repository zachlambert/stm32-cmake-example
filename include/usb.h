#pragma once

#include <libopencm3/usb/usbd.h>

void preinit_usb();
usbd_device* init_usb();
