/* 
 *  video_out_brio.c
 *
 *	Copyright (C) LeapFrog Enterprises, Inc.
 *
 *  This file is part of mpeg2dec, a free MPEG-2 video stream decoder.
 *	
 *  mpeg2dec is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *   
 *  mpeg2dec is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *   
 *  You should have received a copy of the GNU General Public License
 *  along with GNU Make; see the file COPYING.  If not, write to
 *  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA. 
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "config.h"
#include "mp_msg.h"
#include "help_mp.h"
#include "video_out.h"
#include "video_out_internal.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include "mlc_ioctl.h"

static vo_info_t info = 
{
	"Brio YUV video output",
	"brioyuv",
	"LeapFrog Enterprises, Inc. <www.leapfrog.com>",
	""
};

LIBVO_EXTERN(brioyuv)	// DaveM

static uint32_t image_width, image_height;
static int	gDevLayer = -1;
static uint8_t*	gFrameBuffer = NULL;

#define BPP	2

static int draw_slice(uint8_t *image[], int stride[], int w,int h,int x,int y)
{
	// Push pixels from source to dest here
	uint8_t* s = image[0];
	uint8_t* u = image[1];
	uint8_t* v = image[2];
	uint8_t* d = gFrameBuffer + y * (image_width * BPP) + x * BPP;
	int i,j,k,m;

	for (i = 0; i < h; i++) {
		for (j = k = m = 0; k < w; j++, k+=2, m+=4) {
			d[m+0] = s[k];
			d[m+1] = u[j];
			d[m+2] = s[k+1];
			d[m+3] = v[j];
		}
		s += stride[0];
		if (i % 2) {
		u += stride[1];
		v += stride[2];
		}
		d += image_width * BPP;
	}

	return 0;
}

static void draw_osd(void)
{
}

static void
flip_page(void)
{
}

static int
draw_frame(uint8_t *src[])
{
	return 0;
}

static int
query_format(uint32_t format)
{
	// Support any format here
	return VFCAP_CSP_SUPPORTED;
}

static int
config(uint32_t width, uint32_t height, uint32_t d_width, uint32_t d_height, uint32_t flags, char *title, uint32_t format)
{
	int bpp = 2; //4;
	int pitch = width * bpp;
	int hwFormat = 2; //0xC653;
	union mlc_cmd c;

	// Setup any format here
	c.position.top = 0;
	c.position.left = 0;
	c.position.right = c.position.left + d_width;
	c.position.bottom = c.position.top + d_height;
	ioctl(gDevLayer, MLC_IOCSPOSITION, &c);
	c.overlaysize.srcwidth = width;
	c.overlaysize.srcheight = height;
	c.overlaysize.dstwidth = d_width;
	c.overlaysize.dstheight = d_height;
	ioctl(gDevLayer, MLC_IOCSOVERLAYSIZE, &c);
	ioctl(gDevLayer, MLC_IOCTFORMAT, hwFormat);
	ioctl(gDevLayer, MLC_IOCTHSTRIDE, bpp);
	ioctl(gDevLayer, MLC_IOCTVSTRIDE, pitch);
	ioctl(gDevLayer, MLC_IOCTLAYEREN, 1);
	ioctl(gDevLayer, MLC_IOCTDIRTY, 0);

	image_width = width;
	image_height = height;
	return 0;
}

static void
uninit(void)
{
	// Close driver here
	close(gDevLayer);
	printf("vo_brio: close dev/layer = %d\n", gDevLayer);
}


static void check_events(void)
{
}

static int preinit(const char *arg)
{
	int fb_size,baseAddr;

    if(arg) 
    {
	mp_msg(MSGT_VO,MSGL_WARN, MSGTR_LIBVO_NULL_UnknownSubdevice,arg);
	return ENOSYS;
    }
	// Open driver here

	// open MLC YUV layer device
	gDevLayer = open("dev/layer3", O_RDWR|O_SYNC);
	printf("vo_brio: open dev/layer = %d\n", gDevLayer);

	// ask for the Frame Buffer base address
	baseAddr = ioctl(gDevLayer, MLC_IOCQADDRESS, 0);
	printf("vo_brio: ioctl baseAddr = %08X\n", baseAddr);

	// get the frame buffer's size
	fb_size = ioctl(gDevLayer, MLC_IOCQFBSIZE, 0);
	printf("vo_brio: ioctl fb_size = %08X\n", fb_size);

	// get access to the Frame Buffer
	gFrameBuffer = (uint8_t*)mmap(0, fb_size, PROT_READ | PROT_WRITE, MAP_SHARED, gDevLayer, baseAddr);
	printf("vo_brio: mmap frameBuffer = %08X\n", gFrameBuffer);

    return 0;
}

static int control(uint32_t request, void *data, ...)
{
  switch (request) {
  case VOCTRL_QUERY_FORMAT:
    return query_format(*((uint32_t*)data));
  }
  return VO_NOTIMPL;
}
