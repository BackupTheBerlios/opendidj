#CROSS_COMPILE ?= arm-elf-
CROSS_COMPILE ?= arm-linux-uclibc-

CFLAGS=-mcpu=arm926ej-s -mlittle-endian -msoft-float -ffreestanding -Os -Wall
# no spaces between members
LDFLAGS= -static --architecture armv5te -EL -M -Map uniboot.map -nostdlib -e StartUp --section-start .init=0x0000 --use-blx

# mark us as a bootloader to skip kernel-specific stuff
CFLAGS+=-D_LF1000_BOOTLOADER

# XXX: to speed up the bootstrap process, configure PLLs and SDRAM timings
# CFLAGS +=-DSPEEDUP_BOOT

ifneq ($(KERNELDIR),)
CFLAGS+=-I$(KERNELDIR)/include/asm/arch -I$(KERNELDIR)/include/linux
endif

OBJS=startup.o uart.o

ifneq ($(DEBUG),)
CFLAGS+=-DDEBUG
endif

all: $(OBJS)
	$(CROSS_COMPILE)gcc $(CFLAGS) -c uniboot.c
	$(CROSS_COMPILE)ld $(LDFLAGS) $(OBJS) uniboot.o -o uniboot
	$(CROSS_COMPILE)objcopy -S -I elf32-littlearm -O binary uniboot uniboot.bin
	$(CROSS_COMPILE)objdump -D -b binary -EL -m armv5te uniboot.bin > uniboot.dis
	ls -la uniboot.bin

uart.o: uart.c
	$(CROSS_COMPILE)gcc $(CFLAGS) -c uart.c

startup.o: startup.S
	$(CROSS_COMPILE)gcc $(CFLAGS) -Wa,--defsym,StartUp=0x0000 -c startup.S

install: all
	cp uniboot.bin /tftpboot/

clean:
	rm -f *.o *.bin *.dis *.map uniboot autoconf.h

