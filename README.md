# AVR-based event recorder with USB support

Event recorder device based on AVR processor, with the USB support enabled via V-USB library.

* **Project code name**: Fisherman's Friend
* **Homepage**: https://github.com/vladast/avr-based-event-recorder-with-usb-support

## Hardware

Key components of the hardware implementation are:
* **Atmel's Atmega8 processor**: a heart of the device, within which all logic lies.
* **ST's 24c64 EEPROM**: a persistent memory of the device.

Schematic diagram and PCB outline are created with Eagle v6 and can be found within **hw** directory.

## Software

Software part of the project consist of two parts:
* **Firmware**: a program being uploaded to device.
* **Test application**: a program running on host machine.

Programming languages used: C & C++.

Source code is located within **sw** directory.

## Configuration

There are couple of properties that have to be updated within Makefile before build can be started - following values should be set correctly:
* **PROGRAMMER**: By default, it states *rs232* which is the programmer used on test circuit. It should be set to the programmer's name from avrdude's configuration.
* **USB_CFG_VENDOR_ID**: USB VID 16bit value (see Makefile comments for more info, or refer to V-USB's site)
* **USB_CFG_DEVICE_ID**: USB PID 16bit value (see Makefile comments for more info, or refer to V-USB's site)
* **USB_CFG_VENDOR_NAME**: Name of the vendor. By default, for testing purposes, it's set to "TEST"
* **USB_CFG_VENDOR_NAME_LEN**: Number of chars within your vendor's name. For "TEST" string, that number is 4.
* **USB_CFG_DEVICE_NAME**: Name of your product. By default, for testing purposes, it's set to "TEST"
* **USB_CFG_DEVICE_NAME_LEN**: Number of chars within your vendor's name. For "TEST" string, that number is 4.

## Build procedure

At this moment, Makefile has only been intended for usage on Linux-based build machines with gcc and avr-gcc compilers installed. 
In general, with some modifications, it can be used on other platforms.

To build both firmware and test application run:

```
make build
```

To build only device's firmware (HEX file):

```
make build_hex
```

And to build only test application for host machine:

```
make build_test
```

To list all options from Makefile, run:

```
make
```

## Uploading firmware onto device

Used Makefile is configured for usage of *avrdude* programmer. To upload firmware to AVR located on device run:

```
make burn_firmware
```

**Note**: To use external crystal oscillator, for AVR chip used (which is Atmega8 at this moment), fuses has to be set first. To do so, run:

```
make burn_fuses
```

Of course, both of those commands should be issued as is from **sw** directory.

## 3rd party libraries

Following libraries were used:

* **V-USB**: A firmware-only USB driver for Atmel AVR Microcontrollers, enabling USB communication with host device. Library itself and documentation can be found on www.obdev.at/vusb/. 
V-USB is licensed under GPL version 2.
* **24C EEPROM SPI**: Enables easy SPI communication with external EEPROM. It's usage is well described on http://extremeelectronics.co.in/avr-tutorials/easy-24c-i2c-serial-eeprom-interfacing-with-avr-microcontrollers/

## Author and Licence

**avr-based-event-recorder-with-usb-support** repository is maintained by *Vladimir Stankovic*.

This project is licensed under *GPL Version 2*. Please, refer to LICENSE file for the full text of the license.
