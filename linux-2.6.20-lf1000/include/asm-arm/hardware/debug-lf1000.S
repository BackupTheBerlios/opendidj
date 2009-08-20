/* linux/include/asm-arm/hardware/debug-lf1000.S
 *
 * Debugging macro include header
 *
 *  Copyright (C) 2007	Kosta Demirev <kdemirev@yahoo.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/
#include "../arch/platform.h"
#include "../arch/uart.h"

		.macro	waituart,rd,rx
1001:		ldr	\rd, [\rx, #FSTATUS]
		tst	\rd, #(1<<TX_FIFO_FULL)
		bne	1001b
		.endm

		.macro	busyuart,rd,rx
1001:		ldr	\rd, [\rx, #FSTATUS]
		tst	\rd, #(1<<TX_FIFO_FULL)
		bne	1001b
		.endm

		.macro	senduart,rd,rx
		strb	\rd, [\rx, #THB]
		waituart \rd, \rx
		.endm

