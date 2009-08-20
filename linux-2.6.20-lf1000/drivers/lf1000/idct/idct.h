/* LF1000 IDCT Macro Block Decoder
 * 
 * idct.h -- Module and hardware-specific definitions.
 *
 * Andrey Yurovsky <andrey@cozybit.com> */

#ifndef IDCT_H
#define IDCT_H

#include <linux/module.h>
#include <linux/types.h>
#include <asm/io.h>

/* module-related definitions */

#define IDCT_MAJOR 	248	

struct idct_device {
	void *mem;
	struct cdev *cdev;
	dev_t dev;
	int major;
	struct proc_dir_entry *proc;
};

#endif

