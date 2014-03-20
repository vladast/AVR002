/* Name:    usbdefs.h
 * Project: AVR001:	USB touch-switch stats
 * Author:  Vladimir Stankovic
 * Creation Date: 2014-03-09
 * Copyright: N/A
 * License: N/A
 */

#ifndef USBDEFS_H
#define USBDEFS_H

#define VLADAST_VENDOR_NAME     'V', 'l', 'a', 'd', 'a', 's', 't'

/* AVR001 project defines from usbconfig file */
#define AVR001_VENDOR_ID        0xc0, 0x16
#define AVR001_VENDOR_NAME      VLADAST_VENDOR_NAME
#define AVR001_DEVICE_ID        0xe8, 0x03
#define AVR001_DEVICE_NAME      'A', 'V', 'R', '0', '0', '1'

#define USB_CFG_VENDOR_ID       AVR001_VENDOR_ID
#define USB_CFG_VENDOR_NAME     AVR001_VENDOR_NAME
#define USB_CFG_DEVICE_ID       AVR001_DEVICE_ID
#define USB_CFG_DEVICE_NAME     'V', 'l', 'a', 'd', 'a', 's', 't'//AVR001_DEVICE_NAME

#endif // USBDEFS_H
