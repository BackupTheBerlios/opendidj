/* LF1000 3D Engine
 * 
 * ga3d.h -- Module and hardware-specific definitions.
 *
 * Andrey Yurovsky <andrey@cozybit.com> */

#ifndef GA3D_H
#define GA3D_H

#include <linux/module.h>
#include <linux/types.h>
#include <asm/io.h>

/* module-related definitions */

#define GA3D_MAJOR 	249	

struct ga3d_device {
	void *mem;
	struct cdev *cdev;
	dev_t dev;
	int major;
	struct proc_dir_entry *proc;
};

#endif

