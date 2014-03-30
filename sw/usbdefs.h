/*
 * Copyright (C) 2014  Vladimir Stankovic
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#ifndef USBDEFS_H
#define USBDEFS_H

#define VLADAST_VENDOR_NAME     'V', 'l', 'a', 'd', 'a', 's', 't'

/* AVR001 project defines from usbconfig file */
#define AVR001_VENDOR_ID        USB_CFG_VENDOR_ID
#define AVR001_VENDOR_NAME      VLADAST_VENDOR_NAME
#define AVR001_DEVICE_ID        USB_CFG_DEVICE_ID
#define AVR001_DEVICE_NAME      USB_CFG_DEVICE_NAME
#define AVR001_DEVICE_NAME_LEN  USB_CFG_DEVICE_NAME_LEN
#define AVR001_CODE             0xa001

/* AVR002 project defines from usbconfig file */
#define AVR002_VENDOR_ID        USB_CFG_VENDOR_ID
#define AVR002_VENDOR_NAME      VLADAST_VENDOR_NAME
#define AVR002_DEVICE_ID        USB_CFG_DEVICE_ID
#define AVR002_DEVICE_NAME      USB_CFG_DEVICE_NAME
#define AVR002_DEVICE_NAME_LEN  USB_CFG_DEVICE_NAME_LEN
#define AVR002_CODE             0xa002

#define USB_CFG_VENDOR_ID       AVR002_VENDOR_ID
#define USB_CFG_VENDOR_NAME     AVR002_VENDOR_NAME
#define USB_CFG_DEVICE_ID       AVR002_DEVICE_ID
#define USB_CFG_DEVICE_NAME     AVR002_DEVICE_NAME
#define USB_CFG_DEVICE_CODE     AVR002_CODE

#endif // USBDEFS_H
