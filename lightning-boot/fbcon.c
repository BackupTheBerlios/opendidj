/**
 * fbcon.c
 * Frame buffer console driver
 *
 * \author Joe Burks
 */

#include <string.h>

#include "include/common.h"
#include "include/board.h"
#include "include/fbuffer.h"
#include "include/fbcon.h"
#include "include/font.h"

u8 consoleX, consoleY;

/**
   Initialize FrameBuffer. Clears the buffer and puts the cursor to the upper left.
 */
void fbcon_init(void) {
	fbinit();
	fbclear();	
}

/**
   Render a character to the frame buffer at the cursor location.

   \param c A character to render to the frame buffer. 32 <= c <= 127.
 */

void console_putch(char c) {
	if ( c == '\n' ) {
		consoleY++;
	} else if ( c == '\r' ) {
		consoleX = 0;
	} else {
		renderAlNum(consoleX,consoleY,c);
		consoleX++;
	}

	if ( consoleX >= 53 ) {
		consoleY++;
		consoleX = 0;
	}
	if ( consoleY >= 30 ) {
		fbscroll();
		consoleY=29;
	}
}

/**
   Render a string to the frame buffer at the cursor location.

   \param str A null-terminated string of characters to render.
 */
void console_puts(char *str) {
	while ( *str != 0 ) {
		console_putch(*str++);
	}
}
