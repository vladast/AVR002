# 
# Copyright (C) 2014  Vladimir Stankovic
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>

# NOTE:
# This Makefile is built on top of the Makefiles from V-USB bundle.
# For more information on V-USB, please refer to pages and packages
# located on http://www.obdev.at/vusb

# Project specific values
DEVICE          = atmega8
PROGRAMMER      = rs232
PROJECT         = avr-based-usb-event-logger
MAIN            = $(PROJECT)

# V-USB defines
#   LB: low byte value in hex format
#   HB: high byte value in hex format
USB_CFG_DEVICE_ID = LB, HB
USB_CFG_VENDOR_ID = LB, HB

USB_CFG_VENDOR_NAME = 'T', 'E', 'S', 'T'
USB_CFG_VENDOR_NAME_LEN = 4

USB_CFG_DEVICE_NAME = 'T', 'E', 'S', 'T'
USB_CFG_DEVICE_NAME_LEN = 4

# Compiler specific values
F_CPU   = 16000000 # 16MHz CPU frequency - same as xtal's
CFLAGS  = -Iusbdrv -I. -DDEBUG_LEVEL=0
OBJECTS = usbdrv/usbdrv.o usbdrv/usbdrvasm.o usbdrv/oddebug.o ./24c64.o $(MAIN).o
COMPILE = avr-gcc -Wall -Os -DF_CPU=$(F_CPU) \
	$(CFLAGS) -mmcu=$(DEVICE) \
	-DUSB_CFG_DEVICE_ID="$(USB_CFG_DEVICE_ID)" -DUSB_CFG_VENDOR_ID="$(USB_CFG_VENDOR_ID)" \
	-DUSB_CFG_VENDOR_NAME="$(USB_CFG_VENDOR_NAME)" -DUSB_CFG_VENDOR_NAME_LEN=$(USB_CFG_VENDOR_NAME_LEN) \
	-DUSB_CFG_DEVICE_NAME="$(USB_CFG_DEVICE_NAME)" -DUSB_CFG_DEVICE_NAME_LEN=$(USB_CFG_DEVICE_NAME_LEN)

# AVR programmer specific
FUSE_L  = 0xEF          # Use external crystal oscillator
FUSE_H  = 0xD9          # Among other, allow serial programming
AVRDUDE = avrdude -c $(PROGRAMMER) -p $(DEVICE) -P /dev/ttyS0 -b 19200 -v

# Test application params
PROJECT_TEST = $(PROJECT)_test

CC          = g++
USBFLAGS    = `libusb-config --cflags`
USBLIBS     = `libusb-config --libs`
CFLAGS_TEST = $(CPPFLAGS) $(USBFLAGS) -O -g -Wall \
	-DUSB_CFG_DEVICE_ID="$(USB_CFG_DEVICE_ID)" -DUSB_CFG_VENDOR_ID="$(USB_CFG_VENDOR_ID)" \
	-DUSB_CFG_VENDOR_NAME="$(USB_CFG_VENDOR_NAME)" -DUSB_CFG_VENDOR_NAME_LEN=$(USB_CFG_VENDOR_NAME_LEN) \
	-DUSB_CFG_DEVICE_NAME="$(USB_CFG_DEVICE_NAME)" -DUSB_CFG_DEVICE_NAME_LEN=$(USB_CFG_DEVICE_NAME_LEN)
LIBS        = $(USBLIBS)
VUSB        = ./vusb
INCLUDES    = -I$(VUSB)

PROGRAM_TEST = $(PROJECT_TEST)
OBJECTS_TEST = $(VUSB)/opendevice.o $(PROJECT_TEST).o

# If make has been called with no params, print usage
usage:
	@echo "Makefile has following options:"
	@echo "    build_hex           Build HEX file for AVR."
	@echo "    build_test          Build command-line test application."
	@echo "    build               Build both HEX file and test application."
	@echo "    rebuild             Clean object files and then run build procedure."
	@echo "    burn_firmware       Burn the firmware onto AVR."
	@echo "    burn_fuses          Burn the fuses onto AVR."
	@echo "    burn                Burn both fuses and firmware onto AVR."
	@echo "    clean               Clean object files."

build:  build_hex build_test

rebuild: clean build

build_hex: $(MAIN).hex

burn: burn_firmware burn_fuses

# rule for programming fuse bits:
burn_fuses:
	@[ "$(FUSE_H)" != "" -a "$(FUSE_L)" != "" ] || \
		{ echo "*** Edit Makefile and choose values for FUSE_L and FUSE_H!"; exit 1; }
	$(AVRDUDE) -U hfuse:w:$(FUSE_H):m -U lfuse:w:$(FUSE_L):m

# rule for uploading firmware:
burn_firmware: $(MAIN).hex
	$(AVRDUDE) -U flash:w:$(MAIN).hex:i

# rule for deleting dependent files (those which can be built by Make):
clean:
	rm -f $(MAIN).hex $(MAIN).lst $(MAIN).obj $(MAIN).cof $(MAIN).list $(MAIN).map $(MAIN).eep.hex $(MAIN).elf *.o usbdrv/*.o $(MAIN).s usbdrv/oddebug.s usbdrv/usbdrv.s ./24c64.o *.o $(VUSB)/*.o $(PROGRAM_TEST)

# Generic rule for compiling C files:
.c.o:
	$(COMPILE) -c $< -o $@

# Generic rule for assembling Assembler source files:
.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@

# Generic rule for compiling C to assembler, used for debugging only.
.c.s:
	$(COMPILE) -S $< -o $@

# file targets:

# Since we don't want to ship the driver multipe times, we copy it into this project:
usbdrv:
	cp -r ../../../usbdrv .

$(MAIN).elf: usbdrv $(OBJECTS)	# usbdrv dependency only needed because we copy it
	$(COMPILE) -o $(MAIN).elf $(OBJECTS)

$(MAIN).hex: $(MAIN).elf
	rm -f $(MAIN).hex $(MAIN).eep.hex
	avr-objcopy -j .text -j .data -O ihex $(MAIN).elf $(MAIN).hex
	avr-size $(MAIN).hex

# debugging targets:

disasm:	$(MAIN).elf
	avr-objdump -d $(MAIN).elf

cpp:
	$(COMPILE) -E $(MAIN).c

# Test make

build_test: $(PROGRAM_TEST)

$(VUSB)/%.o: $(VUSB)/%.c
	$(CC) $(INCLUDES) $(CFLAGS_TEST) -c $< -o $@

$(PROJECT_TEST).o: $(PROJECT_TEST).cpp
	$(CC) $(INCLUDES) $(CFLAGS_TEST) -c $<

$(PROGRAM_TEST): $(OBJECTS_TEST)
	$(CC) -o $(PROGRAM_TEST) $(OBJECTS_TEST) $(LIBS) $(INCLUDES)

strip: $(PROGRAM_TEST)
	strip $(PROGRAM_TEST)
