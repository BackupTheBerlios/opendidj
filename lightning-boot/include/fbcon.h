/**
 * fbcon.h
 * \author Joe Burks
 */
#ifndef FBCON_H
#define FBCON_H
#include "common.h"

extern void fbcon_init(void);
extern void fbcon_putch(char c);
extern void fbcon_puts(char *str);

#endif

