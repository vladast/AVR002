# Name: Makefile
# Project:          AVR002 - USB touch-switch stats
# Author:           Vladimir Stankovic
# Creation Date:    20140321
# Tabsize:          4
# Copyright:        n/a
# License:          n/a

DEVICE  = atmega8
PROGRAMMER = vladast
PROJECT = avr002
MAIN    = $(PROJECT)
F_CPU   = 16000000
FUSE_L  = 0xE4 # Use internal PLL oscillator
FUSE_H  = 0xD9 # Among other, allow serial programming

AVRDUDE = avrdude -c $(PROGRAMMER) -p $(DEVICE) -P /dev/ttyS0 -b 19200 -v

CFLAGS  = -Iusbdrv -I. -DDEBUG_LEVEL=0
OBJECTS = usbdrv/usbdrv.o usbdrv/usbdrvasm.o usbdrv/oddebug.o $(MAIN).o

COMPILE = avr-gcc -Wall -Os -DF_CPU=$(F_CPU) $(CFLAGS) -mmcu=$(DEVICE)
#COMPILE = avr-c++ -Wall -Os -DF_CPU=$(F_CPU) $(CFLAGS) -mmcu=$(DEVICE)

# symbolic targets:
help:
	@echo "This Makefile has no default rule. Use one of the following:"
	@echo "make hex ....... to build hex"
	@echo "make program ... to flash fuses and firmware"
	@echo "make fuse ...... to flash the fuses"
	@echo "make flash ..... to flash the firmware (use this on metaboard)"
	@echo "make clean ..... to delete objects and hex file"

hex: $(MAIN).hex

program: flash fuse

# rule for programming fuse bits:
fuse:
	@[ "$(FUSE_H)" != "" -a "$(FUSE_L)" != "" ] || \
		{ echo "*** Edit Makefile and choose values for FUSE_L and FUSE_H!"; exit 1; }
	$(AVRDUDE) -U hfuse:w:$(FUSE_H):m -U lfuse:w:$(FUSE_L):m

# rule for uploading firmware:
flash: $(MAIN).hex
	$(AVRDUDE) -U flash:w:$(MAIN).hex:i

# rule for deleting dependent files (those which can be built by Make):
clean:
	rm -f $(MAIN).hex $(MAIN).lst $(MAIN).obj $(MAIN).cof $(MAIN).list $(MAIN).map $(MAIN).eep.hex $(MAIN).elf *.o usbdrv/*.o $(MAIN).s usbdrv/oddebug.s usbdrv/usbdrv.s

# Generic rule for compiling C files:
.c.o:
	$(COMPILE) -c $< -o $@

# Generic rule for assembling Assembler source files:
.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

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
