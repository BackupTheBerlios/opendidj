/* 
 * drivers/lf1000/dpc/dpc.h
 *
 * LF1000 Display Controller (DPC) Driver 
 *
 * Andrey Yurovsky <andrey@cozybit.com> */

#ifndef DPC_H
#define DPC_H

#include <linux/module.h>
#include <linux/types.h>
#include <asm/io.h>
#include "dpc_hal.h"
#include <asm/arch/common.h>

/* module-related definitions */

#define DPC_MAJOR	252

/* device-related definitions */

struct dpc_device {
	void *mem;
	struct cdev *cdev;
	dev_t dev;
	int major;
	struct proc_dir_entry *proc;
	u16 backlight;
};

/* hardware-related definitions */

/* PCLK modes */
enum
{
	PCLKMODE_ONLYWHENCPUACCESS,	/* Operate When CPU Acces */
	PCLKMODE_ALWAYS,		/* Operate Always */
};

/* clock sources */
enum {
	VID_VCLK_SOURCE_PLL0	= 0,
	VID_VCLK_SOURCE_PLL1	= 1,
#ifdef CPU_MF2530F
	VID_VCLK_SOURCE_PLL2	= 2,
	VID_VCLK_SOURCE_PSVCLK	= 3,
	VID_VCLK_SOURCE_nPSVCLK	= 4,
#endif
	VID_VCLK_SOURCE_XTI	= 5,
#ifdef CPU_MF2530F
	VID_VCLK_SOURCE_AVCLK	= 6,
#endif
	VID_VCLK_SOURCE_VCLK2	= 7,	/* clock generator 0 */
};

/* yc orders */
enum {
	VID_ORDER_CbYCrY	= 0,
	VID_ORDER_CrYCbY	= 1,
	VID_ORDER_YCbYCr	= 2,
	VID_ORDER_YCrYCb	= 3
};

/* pad clocks */
enum {
	VID_PADVCLK_VCLK	= 0,
	VID_PADVCLK_nVCLK	= 1,
	VID_PADVCLK_VCLK2	= 2,
	VID_PADVCLK_nVCLK2	= 3
};

/* RGB dithering mode. */
enum DITHER
{
	DITHER_BYPASS		= 0,  /* bypass mode. */
	DITHER_5BIT		= 2,  /* 8 bit -> 5 bit mode. */
	DITHER_6BIT		= 3,  /* 8 bit -> 6 bit mode. */
};

/* video formats */
enum {
	VID_FORMAT_RGB555	= 0,
	VID_FORMAT_RGB565	= 1,
	VID_FORMAT_RGB666	= 2,
	VID_FORMAT_RGB888	= 3,
	VID_FORMAT_MRGB555A	= 4,
	VID_FORMAT_MRGB555B	= 5,
	VID_FORMAT_MRGB565	= 6,
	VID_FORMAT_MRGB666	= 7,
	VID_FORMAT_MRGB888A	= 8,
	VID_FORMAT_MRGB888B	= 9,
	VID_FORMAT_CCIR656	= 10,
	VID_FORMAT_CCIR601A	= 12,
	VID_FORMAT_CCIR601B	= 13,
};

int dpc_SetClock0(u8 source, u8 div, u8 delay, u8 out_inv, u8 out_en);
int dpc_SetClock1(u8 source, u8 div, u8 delay, u8 out_inv);
void dpc_SetClockPClkMode(u8 mode);
void dpc_SetClockEnable(u8 en);
void dpc_SetDPCEnable(void);
int dpc_SetMode(u8 format, u8 interlace, u8 invert_field, u8 rgb_mode,
		u8 swap_rb, u8 ycorder, u8 clip_yc, u8 embedded_sync, u8 clock);
int dpc_SetHSync(u32 avwidth, u32 hsw, u32 hfp, u32 hbp, u8 inv_hsync );
int dpc_SetVSync(u32 avheight, u32 vsw, u32 vfp, u32 vbp, u8 inv_vsync,
		 u32 eavheight, u32 evsw, u32 evfp, u32 evbp );
void dpc_SetVSyncOffset(u16 vss_off, u16 vse_off, u16 evss_off, u16 evse_off);
int dpc_SetDelay(u8 rgb, u8 hs, u8 vs, u8 de, u8 lp, u8 sp, u8 rev);
int dpc_SetDither(u8 r, u8 g, u8 b);
void dpc_SetIntEnb(u8 en);
void dpc_SwapRB(u8 swap);
void dpc_SetContrast(u8 contrast);
void dpc_SetBrightness(u8 brightness);
int dpc_GetContrast(void);
int dpc_GetBrightness(void);

#endif

