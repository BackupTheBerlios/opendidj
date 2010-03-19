/**
 * fbuffer.h
 * \author Joe Burks
 */
#ifndef FBUFFER_H
#define FBUFFER_H
#include "common.h"

extern void fbclear();
extern void fbinit();
extern void renderAlNum(int x, int y, char val );
extern void renderHexNyb(int x, int y, int val );
extern void renderString(int x, int y, char *str);
extern void renderHexByte(int x, int y, int val);
extern void renderHexU32(int x, int y, u32 val);
extern void dumpData(u8 *data, int size);
extern void fbscroll();

#endif

