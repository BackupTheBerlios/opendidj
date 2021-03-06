/* 
 * drivers/lf1000/dpc/dpc.c
 *
 * LF1000 Display Controller (DPC) Driver 
 *
 * Copyright 2007 LeapFrog Enterprises Inc.
 *
 * Andrey Yurovsky <andrey@cozybit.com> 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 * */

#include <linux/ioport.h>
#include <linux/types.h>
#include <asm/io.h>
#include <asm/arch/spi.h>

#include "dpc.h"
#include "ili9322.h" /* LCD controller */

extern struct dpc_device dpc;

int dpc_SetClock0(u8 source, u8 div, u8 delay, u8 out_inv, u8 out_en)
{
	void *base = dpc.mem;
	u32 tmp;

	if(source > 7 || delay > 6)
		return -EINVAL;

	tmp = ioread32(base+DPCCLKGEN0);
	tmp &= ~((7<<CLKSRCSEL0)|(0x3F<<CLKDIV0)|(3<<OUTCLKDELAY0));

	tmp |= (source<<CLKSRCSEL0);	/* clock source */
	tmp |= ((0x3F&div)<<CLKDIV0);	/* clock divider */
	tmp |= (delay<<OUTCLKDELAY0);	/* output clock delay */

	out_inv ? BIT_SET(tmp,OUTCLKINV0) : BIT_CLR(tmp,OUTCLKINV0);
	out_en ? BIT_SET(tmp,OUTCLKENB) : BIT_CLR(tmp,OUTCLKENB);

	iowrite32(tmp,base+DPCCLKGEN0);
	return 0;
}

void dpc_SetClockPClkMode(u8 mode)
{
	void *base = dpc.mem;
	u32 tmp = ioread32(base+DPCCLKENB);

	mode ? BIT_SET(tmp,_PCLKMODE) : BIT_CLR(tmp,_PCLKMODE);
	iowrite32(tmp,base+DPCCLKENB);
}

int dpc_SetClock1( u8 source, u8 div, u8 delay, u8 out_inv )
{
	void *base = dpc.mem;
	u32 tmp;

	if( source > 7 || delay > 6 )
		return -EINVAL;

	tmp = ioread32(base+DPCCLKGEN1);
	tmp &= ~((7<<CLKSRCSEL1)|(0x3F<<CLKDIV1)|(3<<OUTCLKDELAY1));

	tmp |= (source<<CLKSRCSEL1);	/* clock source */
	tmp |= ((0x3F&div)<<CLKDIV1);	/* clock divider */
	tmp |= (delay<<OUTCLKDELAY1);	/* output clock delay */
	out_inv ? BIT_SET(tmp,OUTCLKINV1) : BIT_CLR(tmp,OUTCLKINV1);

	iowrite32(tmp,base+DPCCLKGEN1);
	return 0;
}

void dpc_SetClockEnable(u8 en)
{
	void *base = dpc.mem;
	u32 tmp = ioread32(base+DPCCLKENB);

	en ? BIT_SET(tmp,_CLKGENENB) : BIT_CLR(tmp,_CLKGENENB);
	iowrite32(tmp,base+DPCCLKENB);
}

void dpc_SetDPCEnable(void)
{
	void *base = dpc.mem;
	u16 tmp = ioread16(base+DPCCTRL0);

	BIT_SET(tmp,DPCENB);
	BIT_CLR(tmp,_INTENB); /* disable VSYNC interrupt */
	iowrite16(tmp,base+DPCCTRL0);
}

void dpc_SwapRB(u8 swap)
{
	u16 tmp = ioread16(dpc.mem+DPCCTRL1);

	swap ? BIT_SET(tmp,SWAPRB) : BIT_CLR(tmp,SWAPRB);
	iowrite16(tmp, dpc.mem+DPCCTRL1);
}

void dpc_SetContrast(u8 contrast)
{
	lf1000_spi_xfer(LCD_SET((CMD_CONTRAST|(contrast & 0xF))));
}

int dpc_GetContrast(void)
{
	return (lf1000_spi_xfer(LCD_GET(CMD_CONTRAST)) & 0xF);
}

void dpc_SetBrightness(u8 brightness)
{
	lf1000_spi_xfer(LCD_SET((CMD_BRIGHTNESS|(brightness & 0xFF))));
}

int dpc_GetBrightness(void)
{
	return (lf1000_spi_xfer(LCD_GET(CMD_BRIGHTNESS)) & 0xFF);
}

int dpc_SetMode(u8 format,
		u8 interlace,
		u8 invert_field,
		u8 rgb_mode,
		u8 swap_rb,
		u8 ycorder,
		u8 clip_yc,
		u8 embedded_sync,
		u8 clock)
{
	void *base = dpc.mem;
	u16 tmp;

	if(format >= 14 || ycorder > 3 || clock > 3)
		return -EINVAL;

	/* DPC Control 0 Register */
	
	tmp = ioread16(base+DPCCTRL0);
	BIT_CLR(tmp,_INTPEND);

	/* set flags */
	interlace ? BIT_SET(tmp,SCANMODE) : BIT_CLR(tmp,SCANMODE);
	invert_field ? BIT_SET(tmp,POLFIELD) : BIT_CLR(tmp,POLFIELD);
	rgb_mode ? BIT_SET(tmp,RGBMODE) : BIT_CLR(tmp,RGBMODE);
	embedded_sync ? BIT_SET(tmp,SEAVENB) : BIT_CLR(tmp,SEAVENB);

	iowrite16(tmp,base+DPCCTRL0);

	/* DPC Control 1 Register */

	tmp = ioread16(base+DPCCTRL1);
	tmp &= ~(0xAFFF);  /* clear all fields except reserved bits */ 
	tmp |= ((ycorder<<YCORDER)|(format<<FORMAT));
	clip_yc ?  BIT_CLR(tmp,YCRANGE) : BIT_SET(tmp,YCRANGE);
	swap_rb ? BIT_SET(tmp,SWAPRB) : BIT_CLR(tmp,SWAPRB);
	iowrite16(tmp,base+DPCCTRL1);

	/* DPC Control 2 Register */

	tmp = ioread16(base+DPCCTRL2);
	tmp &= ~(3<<PADCLKSEL);
	tmp |= (clock<<PADCLKSEL);
	iowrite16(tmp,base+DPCCTRL2);

	return 0;
}

int dpc_SetHSync(u32 avwidth, u32 hsw, u32 hfp, u32 hbp, u8 inv_hsync)
{
	void *base = dpc.mem;
	u16 tmp;

	if( avwidth + hfp + hsw + hbp > 65536 || hsw == 0 )
		return -EINVAL;

	iowrite16((u16)(hsw+hbp+hfp+avwidth-1),base+DPCHTOTAL);
	iowrite16((u16)(hsw-1),base+DPCHSWIDTH);
	iowrite16((u16)(hsw+hbp-1),base+DPCHASTART);
	iowrite16((u16)(hsw+hbp+avwidth-1),base+DPCHAEND);

	tmp = ioread16(base+DPCCTRL0);
	BIT_CLR(tmp,_INTPEND);
	if(inv_hsync)
		BIT_SET(tmp,POLHSYNC);
	else
		BIT_CLR(tmp,POLHSYNC);
	iowrite16(tmp,base+DPCCTRL0);

	return 0;
}

int dpc_SetVSync(u32 avheight, u32 vsw, u32 vfp, u32 vbp, u8 inv_vsync,
		u32 eavheight, u32 evsw, u32 evfp, u32 evbp)
{
	void *base = dpc.mem;
	u16 tmp;

	if( avheight+vfp+vsw+vbp > 65536 || avheight+evfp+evsw+evbp > 65536 ||
		vsw == 0 || evsw == 0 )
		return -EINVAL;

	iowrite16((u16)(vsw+vbp+avheight+vfp-1),base+DPCVTOTAL);
	iowrite16((u16)(vsw-1),base+DPCVSWIDTH);
	iowrite16((u16)(vsw+vbp-1),base+DPCVASTART);
	iowrite16((u16)(vsw+vbp+avheight-1),base+DPCVAEND);

	iowrite16((u16)(evsw+evbp+eavheight+evfp-1),base+DPCEVTOTAL);
	iowrite16((u16)(evsw-1),base+DPCEVSWIDTH);
	iowrite16((u16)(evsw+evbp-1),base+DPCEVASTART);
	iowrite16((u16)(evsw+evbp+eavheight-1),base+DPCEVAEND);

	tmp = ioread16(base+DPCCTRL0);
	BIT_CLR(tmp,_INTPEND);
	inv_vsync ? BIT_SET(tmp,POLVSYNC) : BIT_CLR(tmp,POLVSYNC);
	iowrite16(tmp,base+DPCCTRL0);
	return 0;
}

void dpc_SetVSyncOffset(u16 vss_off, u16 vse_off, u16 evss_off, u16 evse_off)
{
	iowrite16(vse_off,dpc.mem+DPCVSEOFFSET);
	iowrite16(vss_off,dpc.mem+DPCVSSOFFSET);
	iowrite16(evse_off,dpc.mem+DPCEVSEOFFSET);
	iowrite16(evss_off,dpc.mem+DPCEVSSOFFSET);
}

int dpc_SetDelay(u8 rgb, u8 hs, u8 vs, u8 de, u8 lp, u8 sp, u8 rev)
{
	void *base = dpc.mem;
	u16 tmp;

	if(rgb>=16 || hs>=16 || vs>=16 || de>=16 || lp>=16 || sp>=16 || rev>=16 )
		return -EINVAL;

	tmp = ioread16(base+DPCCTRL0);
	tmp &= ~((1<<_INTPEND)|(0xF<<DELAYRGB));
	tmp |= (rgb<<DELAYRGB);
	iowrite16(tmp,base+DPCCTRL0);

	iowrite16((u16)((de<<DELAYDE)|(vs<<DELAYVS)|(hs<<DELAYHS)),
				base+DPCDELAY0);
#ifdef CPU_MF2530F
	iowrite16((u16)((rev<<DELAYREV)|(sp<<DELAYSP)|(lp<<DELAYLP)),
				base+DPCDELAY1);
#endif
	return 0;
}

int dpc_SetDither(u8 r, u8 g, u8 b)
{
	u16 tmp;

	if(r >= 4 || g >= 4 || b >= 4)
		return -EINVAL;

	tmp = ioread16(dpc.mem+DPCCTRL1);
	tmp &= ~(0x3F);
	tmp |= ((r<<RDITHER)|(g<<GDITHER)|(b<<BDITHER));
	iowrite16(tmp,dpc.mem+DPCCTRL1);
	return 0;
}

void dpc_SetIntEnb(u8 en)
{
	void *base = dpc.mem;
	u16 tmp = ioread16(base+DPCCTRL0);

	if(en)
		BIT_SET(tmp,_INTENB);
	else {
		BIT_CLR(tmp,_INTENB);
		BIT_CLR(tmp,_INTPEND);
	}
	iowrite16(tmp,base+DPCCTRL0);
}
