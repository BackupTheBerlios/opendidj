#include "autoconf.h"
#include "mach-types.h"
#include <platform.h>
#include <common.h>
#include <uart.h>
#include <nand.h>
#include <gpio.h>
#include <gpio_hal.h>
#include <clkpwr.h>

#include "uart.h"
#include "nand.h"
#include "pll.h"

typedef void (app)(void);

static void nand_read(u32 *dest, u32 offset, u32 size, char flags)
{
	u32 page, page_size_w;
	u32 size_w = (size>>2);
	int i;

	if(flags & (1<<NAND_LARGE_BLOCK)) {
		page = (offset>>11);
		page_size_w = (2048>>2);
	}
	else {
		page = (offset>>9);
		page_size_w = (512>>2);
	}

	while(1) {
		/* start a page read */
		REG8(LF1000_NAND_BASE+NFCMD) = CMD_READ;

		/* send the column and page address */
		REG8(LF1000_NAND_BASE+NFADDR) = 0;
		if(flags & (1<<NAND_LARGE_BLOCK)) {
			REG8(LF1000_NAND_BASE+NFADDR) = 0;
		}
		REG8(LF1000_NAND_BASE+NFADDR) = (u8)page;
		REG8(LF1000_NAND_BASE+NFADDR) = (u8)(page>>8);
		if(flags & (1<<NAND_EXTRA_ADDRESS)) {
			REG8(LF1000_NAND_BASE+NFADDR) = (u8)(page>>16);
		}

		if(flags & (1<<NAND_LARGE_BLOCK))
			REG8(LF1000_NAND_BASE+NFCMD) = CMD_READ_CONFIRM;

		/* wait for the chip to be ready */
		REG8(LF1000_NAND_BASE+NFCMD)= CMD_READ_STATUS;
		while(!(REG8(LF1000_NAND_BASE+NFDATA) & (1<<NAND_STATUS_READY)));

		/* read the page into RAM */
		REG8(LF1000_NAND_BASE+NFCMD) = CMD_READ;
		for(i = 0; i < page_size_w; i++) {
			*dest++ = REG32(LF1000_NAND_BASE+NFDATA);
			size_w--;
			if(size_w == 0) return;
		}
		page++;
	}
}

void main(void)
{
	u32 status;
	u32 start, size;
	u8 nand_flags = 0;

#ifdef CPU_LF1000
	/* enable access to Alive GPIO */
	REG32(LF1000_ALIVE_BASE+ALIVEPWRGATEREG) = 1;
	/* pull VDDPWRON high by setting the flip-flop */
	REG32(LF1000_ALIVE_BASE+ALIVEGPIOSETREG) |= (1<<7);
	/* reset flip-flop to latch in */
	REG32(LF1000_ALIVE_BASE+ALIVEGPIOSETREG) = 0;
	REG32(LF1000_ALIVE_BASE+ALIVEGPIORSTREG) = 0;

	/* set up nSCS2 (chip select) for Ethernet */
	REG32(LF1000_GPIO_BASE+GPIOCALTFN0) &= ~(0x3<<GPIO_PIN15*2);
	REG32(LF1000_GPIO_BASE+GPIOCALTFN0) |= (GPIO_ALT1<<GPIO_PIN15*2);
#endif

#ifdef DEBUG
	/* LEDs: GPIOA-28 through GPIOA-31, active low */
	REG32(LF1000_GPIO_BASE+GPIOAOUTENB) = ((1<<31)|(1<<30)|(1<<29)|(1<<28));
#endif

#ifdef SPEEDUP_BOOT
	/* Kosta: With 340 Mhz PLL0 and NAND Flash CS timings bellow I can have
	 * 720ms boot time for uniboot loading 2.5 Mbytes from Flash.  With
	 * default settings boot time is ~ 940ms. */

	/* set SDRAM and BCLK clock to PLL1 */
	REG32(LF1000_CLKPWR_BASE+CLKMODEREG) = 0x00306840
		#if defined CPU_MP2530F
		 | (SDRAM_PLL<<CLKSELSDRAM)
		#endif
		 | (BCLK_PLL<<CLKSELBCLK);
	/* set max freq for all LF boards (400MHz) */
	REG32(LF1000_CLKPWR_BASE+PLLSETREG0) =	(PDIV_UNI<<PDIV_0) |
			     			(MDIV_UNI<<MDIV_0) |
			     			(SDIV_UNI<<SDIV_0);
	/* apply */
	REG32(LF1000_CLKPWR_BASE+PWRMODE) = (1<<CHGPLL);

	/* wait for PLLs to stabalize */
	do{
	    __asm__ __volatile__ ("nop	;\n");
	} while( REG32(LF1000_CLKPWR_BASE+PWRMODE) & (1<<CHGPLL));

	/* change nand CS timings, all hex numbers are reset default value for
	 * MF2530F, field width is 2 or 4 [(NAND_CS*2) or (NAND_CS*4)] */
	REG32(LF1000_MEMTIMEACS_PHY) = 0x00000005|(LF1000_MEMTIMEACS_VAL <<(NAND_CS*2));
	REG32(LF1000_MEMTIMECOS_PHY) = 0x00000005|(LF1000_MEMTIMECOS_VAL <<(NAND_CS*2));
	REG32(LF1000_MEMTIMECOH_PHY) = 0x00000005|(LF1000_MEMTIMECOH_VAL <<(NAND_CS*2));
	REG32(LF1000_MEMTIMECAH_PHY) = 0x00000005|(LF1000_MEMTIMECAH_VAL <<(NAND_CS*2));
	REG32(LF1000_MEMTIMEACCL_PHY) = 0x0000004F|(LF1000_MEMTIMEACCL_VAL<<(NAND_CS*4));
#endif /* SPEEDUP_BOOT */

#ifdef CPU_LF1000
	/* slow down the device to account for the faster clock (per Sam) */
	REG32(LF1000_MEMTIMEACS_PHY)  = REG32(LF1000_MEMTIMEACS_PHY)  | 0x00400000;
	REG32(LF1000_MEMTIMECOS_PHY)  = REG32(LF1000_MEMTIMECOS_PHY)  | 0x00400000;
	REG32(LF1000_MEMTIMECOH_PHY)  = REG32(LF1000_MEMTIMECOH_PHY)  | 0x00400000;
	REG32(LF1000_MEMTIMECAH_PHY)  = REG32(LF1000_MEMTIMECAH_PHY)  | 0x00400000;
	REG32(LF1000_MEMTIMEACCH_PHY) = REG32(LF1000_MEMTIMEACCH_PHY) | 0x00006000;
#endif /* CPU_LF1000 */

	uart_init();
	uart_puts("LF1000 Bootstrap 1.0\r\n");

	status = REG32(LF1000_MCU_S_BASE+NFCONTROL);
	if(status & (1<<NFTYPE_LBLOCK)) {
		uart_puts("\tusing Large Block NAND\r\n");
		nand_flags |= (1<<NAND_LARGE_BLOCK);
	}
	else {
		uart_puts("\tusing Small Block NAND\r\n");
	}

	if(status & (1<<NFTYPE_EADDR)) {
		nand_flags |= (1<<NAND_EXTRA_ADDRESS);
		uart_puts("\tusing extra address byte\r\n");
	}

	start = KERNEL_IMAGE_START;
	size  = KERNEL_IMAGE_SIZE;

#ifndef CONFIG_MACH_LF_LF1000
	/* mask for whatever buttons are attched to PORTC */
#ifdef CONFIG_MACH_ME_LF1000
	#define BUTTON_MSK	0x00003F9F
#else /* LF_ME2530 and ME_ME2530 boards */
	#define	BUTTON_MSK	0x1FE00000
#endif
	if(((REG32(LF1000_GPIO_BASE+GPIOCPAD) & BUTTON_MSK) != BUTTON_MSK)) {
		start = UBOOT_IMAGE_START;
		/* lets load for both images same size
		u-boot does not care( speed and size wise)
		size  = UBOOT_IMAGE_SIZE; */
		uart_puts("in uboot mode\r\n");
	}
#endif /* !CONFIG_MACH_LF_LF1000 */

	nand_read((u32 *)RAM_IMAGE_START, start, size, nand_flags);
	uart_puts("NAND image read into RAM, booting...\r\n");

#ifdef DEBUG
	/* turn off LEDs to indicate that we are leaving bootloader */
	REG32(LF1000_GPIO_BASE+GPIOAOUT) |= ((1<<31)|(1<<30)|(1<<29)|(1<<28));
#endif

	/* jump to image */
	// 3 parameters : void, architecture ID, atags pointer
	//		   r0          r1             r2
	(( void (*)( int r0, int r1, int r2))RAM_IMAGE_START) ( 0, MACH_TYPE_LF1000, 0);

	/* never get here! */
}
