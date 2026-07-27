##############################################################################
# AVR Makefile (Linux, selectable clock)
##############################################################################

DEVICE     = attiny85

# -------- Clock selection --------
# Usage:
#   make CLOCK_MODE=4   (default, external crystal)
#   make CLOCK_MODE=8   (internal)


CLOCK_MODE ?= 4

ifeq ($(CLOCK_MODE),4)
    CLOCK = 4000000
    FUSES = -U lfuse:w:0xFF:m -U hfuse:w:0xD7:m -U efuse:w:0xFF:m
else
    CLOCK = 8000000
    FUSES = -U lfuse:w:0xFD:m -U hfuse:w:0xD7:m -U efuse:w:0xFF:m
endif

# -------- Programmer --------
PROGRAMMER = -c usbasp

# -------- Sources --------
SRCS = main.c dtmf.c
OBJS = $(SRCS:.c=.o)

# -------- Tools --------
CC       = avr-gcc
OBJCOPY  = avr-objcopy
OBJDUMP  = avr-objdump
AVRDUDE  = avrdude $(PROGRAMMER) -p $(DEVICE)

CFLAGS = -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)

# -------- Targets --------

all: retrodial.hex

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

retrodial.elf: $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS)

retrodial.hex: retrodial.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

flash: all
	$(AVRDUDE) -U flash:w:retrodial.hex:i

fuse:
	$(AVRDUDE) $(FUSES)

erase:
	dd if=/dev/zero bs=512 count=1 | tr '\000' '\377' > /tmp/eeprom_blank.bin
	$(AVRDUDE) -U eeprom:w:/tmp/eeprom_blank.bin:r
	rm /tmp/eeprom_blank.bin

install: clean flash fuse

clean:
	rm -f *.o *.elf *.hex

disasm: retrodial.elf
	$(OBJDUMP) -d $<

cpp:
	$(CC) $(CFLAGS) -E $(SRCS)
