/**
 * fbuffer.c
 * Frame buffer driver
 *
 * \author Joe Burks
 */
#include <string.h>
#include "include/board.h"
#include "include/common.h"
#include "include/fbcon.h"
#include "include/font.h"

u8 bgcolor[3];
u8 fgcolor[3];
u8 opaque;

/**
  Initialize framebuffer internal state.
 */
void fbinit() {
	bgcolor[0] = 0x00;
	bgcolor[1] = 0x00;
	bgcolor[2] = 0xFF;

	fgcolor[0] = 0xFF;
	fgcolor[1] = 0xFF;
	fgcolor[2] = 0xFF;

	opaque = 1;
}

/**
  Erase the framebuffer using background color.
 */
void fbclear() {
	int i,k;
	u8 *fb_addr;
	fb_addr = (u8 *)(FRAME_BUFFER_ADDR);
	for ( i = 0, k = 0; i < 320*240*3; i++ ) {
		char val = bgcolor[k++];
		if ( k == 3 ) k = 0;
		*fb_addr++ = val;
	}
}

/**
  Scroll the display one line.
  Note: this is coupled to the notion of an 8 pixel tall font. This should get fixed at some point.
 */
void fbscroll() {
	int i,k;
	u8 *fb_addr;

	memcpy ((void *)(FRAME_BUFFER_ADDR),(void *)(FRAME_BUFFER_ADDR + 320*8*3),320*(240-8)*3);

	fb_addr = (u8 *)(FRAME_BUFFER_ADDR + 320*(240-8)*3);
	for ( i = 0, k = 0; i < 320*8*3; i++ ) {
		char val = bgcolor[k++];
		if ( k == 3 ) k = 0;
		*fb_addr++ = val;
	}
}

/**
  Render an alphanumeric value (actually any ascii value between 32 and 127 inclusive).
  Note: This function assumes coordinate system for a 6x8 font. This should be fixed at some point.

  \param x The X text coordinate of the letter
  \param y The Y text coordinate of the letter
  \param val The character to render
 */
void renderAlNum(int x, int y, char val ) {
    int l,b;
	int off;
	off = val - ' ';
	u8* fbaddr = (u8 *)(FRAME_BUFFER_ADDR + y*8*320*3 + x*6*3);
	for ( l = 0; l < 8; l++ ) {
		u8 line = font[off*8+l] >> 1;
		for ( b = 0; b < 6; b++ ) {
			if ( (line & 128) != 0 ) { fbaddr[0] = fgcolor[0]; fbaddr[1] = fgcolor[1]; fbaddr[2] = fgcolor[2]; }
			else if (opaque) { fbaddr[0] = bgcolor[0]; fbaddr[1] = bgcolor[1]; fbaddr[2] = bgcolor[2]; }
			line <<= 1;
			fbaddr+=3;
		}
		fbaddr += 320*3 - 18;
	}
}

/**
  Render a 4-bit hexadecimal "Nybble" to the display.
  Note: This function assumes coordinate system for a 6x8 font. This should be fixed at some point.

  \param x The X text coordinate of the hex digit.
  \param y The Y text coordinate of the hex digit.
  \param val The value to render. 0 <= val <= 15
 */
void renderHexNyb(int x, int y, int val ) {
	if ( val < 0xA ) renderAlNum(x,y,val+'0');
	else renderAlNum(x,y,(val-0xA)+'A');
}

/**
  Render a null terminate string of characters. No intelligent wrapping is performed.
  Note: This function assumes coordinate system for a 6x8 font. This should be fixed at some point.

  \param x The X text coordinate of the hex digit.
  \param y The Y text coordinate of the hex digit.
  \param str The null-terminated string to render.
 */
void renderString(int x, int y, char *str) {
	while ( *str != 0 ) {
		renderAlNum(x,y,*str);
		str++;
		x++;
	}
}

/**
  Render a byte value as two hex digits.
  Note: This function assumes coordinate system for a 6x8 font. This should be fixed at some point.

  \param x The X text coordinate of the hex byte
  \param y The Y text coordinate of the hex byte
  \param val The byte to write (only low order 8 bits will be rendered)
 */
void renderHexByte(int x, int y, int val) {
	renderHexNyb(x,y,(val>>4)&0xF);
	renderHexNyb(x+1,y,(val)&0xF);
}

/**
  Render a dword as 8 hex digits.
  Note: This function assumes coordinate system for a 6x8 font. This should be fixed at some point.

  \param x The X text coordinate of the hex byte
  \param y The Y text coordinate of the hex byte
  \param val The dword to write
 */
void renderHexU32(int x, int y, u32 val) {
	renderHexByte(x,y,val>>24);
	renderHexByte(x+2,y,(val>>16)&0xFF);
	renderHexByte(x+4,y,(val>>8)&0xFF);
	renderHexByte(x+6,y,(val>>0)&0xFF);

}

/**
  Write a block of data as hex bytes. This method is here for debugging.
  The first line will contain the number of bytes written, the next lines
  will contain the data written 16 bytes per line. The upper limit for
  size is 16*29.

  \param data Pointer to bytes to dump to display
  \param size The number of bytes to dump.
 */
void dumpData(u8 *data, int size) {
	int i,j;
	renderHexByte(0,0,size);
	for ( i = 0; i < size; i+=16 ) {
		for ( j = 0; (j < 16) && ((j+i) < size); j++ ) {
			renderHexByte(j*3,(i>>4)+1,data[i+j]);
		}
	}
}
