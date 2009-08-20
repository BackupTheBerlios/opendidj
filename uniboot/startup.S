/*
 *  uniboot startup.S
 *
 *      Copyright (C) 2007 Kosta Demirev <kdemirev@yahoo.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
.section .init
.arm
.align 0

	.global StartUp

/*------------------------------------------------------------------------------*/
/* Entry Point : StartUp							*/
/*------------------------------------------------------------------------------*/
	.org	StartUp

ResetHandler:
	mov		sp,	#0x00010000	/* This Code must be removed. */
	b		main

.end
