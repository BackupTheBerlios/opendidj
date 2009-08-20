/* 
 * drivers/lf1000/mlc/main.c
 *
 * LF1000 Multi-Layer Controller (MLC) Driver 
 *
 * Copyright 2007 LeapFrog Enterprises Inc.
 *
 * Andrey Yurovsky <andrey@cozybit.com>
 *
 * TODO: - kernel frame buffer support
 *	 - integrate with DPC driver?
 *	 - move to drivers/video
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/lf1000/mlc_ioctl.h>

#include <asm/arch/platform.h>
#include <asm/arch/common.h>
#include <asm/arch/mlc.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include "mlc_hal.h"
#include "mlc_priv.h"

/* device private data */
struct mlc_device mlc = {
	.mem = NULL,
};

/* layer organization */
#ifdef CPU_LF1000
unsigned int fbaddr[MLC_NUM_LAYERS] = {RGB_BASE, OGL_BASE, YUV_BASE};
unsigned int fbsize[MLC_NUM_LAYERS] = {RGB_SIZE, OGL_SIZE, YUV_SIZE};
#else
unsigned int fbaddr[MLC_NUM_LAYERS] = {RGB_BASE, OGL_BASE, RGB_BASE, YUV_BASE};
unsigned int fbsize[MLC_NUM_LAYERS] = {RGB_SIZE, OGL_SIZE, RGB_SIZE, YUV_SIZE};
#endif

/*******************
 * sysfs Interface *
 *******************/

#ifdef CONFIG_LF1000_MLC_DEBUG
static ssize_t show_top_registers(struct device *dev, 
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	
	len += sprintf(buf+len,"MLCCONTROLT   = 0x%08X\n",
				   ioread32(mlc.mem+MLCCONTROLT));
	len += sprintf(buf+len,"MLCSCREENSIZE = 0x%08X\n",
				   ioread32(mlc.mem+MLCSCREENSIZE));
	len += sprintf(buf+len,"MLCBGCOLOR    = 0x%08X\n",
				   ioread32(mlc.mem+MLCBGCOLOR));
	len += sprintf(buf+len,"MLCCLKENB     = 0x%08X\n", 
				   ioread32(mlc.mem+MLCCLKENB));

	return len; 
}
static DEVICE_ATTR(registers_top, S_IRUSR|S_IRGRP, show_top_registers, NULL);

static ssize_t show_layer_registers(struct device *dev, 
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	char x;
	int layer;
	void *base = mlc.mem;

	for(layer = 0; layer < MLC_NUM_LAYERS; layer++) {
		x = layer+'0';
#if defined CPU_MF2530F
		/* layer registers are evenly spaced */
		base += layer*0x20;
		len += sprintf(buf+len,"MLCLEFTOP%c      = 0x%08X\n", x,
					   ioread32(base+MLCLEFTOP0));
		len += sprintf(buf+len,"MLCRIGHTBOTTOM%c = 0x%08X\n", x,
					   ioread32(base+MLCRIGHTBOTTOM0));
		len += sprintf(buf+len,"MLCCONTROL%c     = 0x%08X\n", x,
					   ioread32(base+MLCCONTROL0));
		len += sprintf(buf+len,"MLCHSTRIDE%c     = 0x%08X\n", x,
					   ioread32(base+MLCHSTRIDE0));
		len += sprintf(buf+len,"MLCVSTRIDE%c     = 0x%08X\n", x,
					   ioread32(base+MLCVSTRIDE0));
		len += sprintf(buf+len,"MLCTPCOLOR%c     = 0x%08X\n", x,
					   ioread32(base+MLCTPCOLOR0));
		len += sprintf(buf+len,"MLCINVCOLOR%c    = 0x%08X\n", x,
					   ioread32(base+MLCINVCOLOR0));
		len += sprintf(buf+len,"MLCADDRESS%c     = 0x%08X\n", x,
					   ioread32(base+MLCADDRESS0));
#elif defined CPU_LF1000
		/* LF1000 layer registers are not evenly spaced */
		switch (layer) {
		case 0:
			len += sprintf(buf+len,"MLCCONTROL%c     = 0x%08X\n", 
					x, ioread32(base+MLCCONTROL0));
			len += sprintf(buf+len,"MLCHSTRIDE%c     = 0x%08X\n", 
					x, ioread32(base+MLCHSTRIDE0));
			len += sprintf(buf+len,"MLCVSTRIDE%c     = 0x%08X\n", 
					x, ioread32(base+MLCVSTRIDE0));
			len += sprintf(buf+len,"MLCADDRESS%c     = 0x%08X\n", 
					x, ioread32(base+MLCADDRESS0));
			break;
		case 1:
			len += sprintf(buf+len,"MLCCONTROL%c     = 0x%08X\n", 
					x, ioread32(base+MLCCONTROL1));
			len += sprintf(buf+len,"MLCHSTRIDE%c     = 0x%08X\n", 
					x, ioread32(base+MLCHSTRIDE1));
			len += sprintf(buf+len,"MLCVSTRIDE%c     = 0x%08X\n", 
					x, ioread32(base+MLCVSTRIDE1));
			len += sprintf(buf+len,"MLCADDRESS%c     = 0x%08X\n", 
					x, ioread32(base+MLCADDRESS1));
			break;
		case 2:
			len += sprintf(buf+len,"MLCCONTROL%c     = 0x%08X\n", 
					x, ioread32(base+MLCCONTROL2));
			len += sprintf(buf+len,"MLCVSTRIDE%c     = 0x%08X\n", 
					x, ioread32(base+MLCVSTRIDE3));
			len += sprintf(buf+len,"MLCADDRESS%c     = 0x%08X\n", 
					x, ioread32(base+MLCADDRESS3));
			break;
		}
#endif /* CPU_LF1000 */
	}

	return len;
}
static DEVICE_ATTR(registers_layer, S_IRUSR|S_IRGRP, 
				show_layer_registers, NULL);


static struct attribute *mlc_attributes[] = {
	&dev_attr_registers_top.attr,
	&dev_attr_registers_layer.attr,
	NULL
};

static struct attribute_group mlc_attr_group = {
	.attrs = mlc_attributes
};
#endif /* CONFIG_LF1000_MLC_DEBUG */

/*******************************
 * character device operations *
 *******************************/

int mlc_open(struct inode *inode, struct file *filp)
{
	return 0;
}

int mlc_layer_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
					unsigned long arg)
{
	int retval = 0;
	void __user *argp = (void __user *)arg;
	union mlc_cmd c;
	struct mlc_layer *layer;

	layer = container_of(inode->i_cdev, struct mlc_layer, dev);

	switch(cmd) {
		case MLC_IOCTLAYEREN:
		retval = mlc_SetLayerEnable(layer->id, arg);
		break;

		case MLC_IOCTADDRESS:
		retval = mlc_SetAddress(layer->id, arg);
		break;

		case MLC_IOCTADDRESSCB:
		retval = mlc_SetAddressCb(layer->id, arg);
		break;

		case MLC_IOCTADDRESSCR:
		retval = mlc_SetAddressCr(layer->id, arg);
		break;

		case MLC_IOCTLOCKSIZE:
		retval = mlc_SetLockSize(layer->id, arg);
		break;

		case MLC_IOCTHSTRIDE:
		retval = mlc_SetHStride(layer->id, arg);
		break;

		case MLC_IOCTVSTRIDE:
		retval = mlc_SetVStride(layer->id, arg);
		break;

		case MLC_IOCT3DENB:
		retval = mlc_Set3DEnable(layer->id, arg);
		break;

		case MLC_IOCTALPHA:
		retval = mlc_SetTransparencyAlpha(layer->id, arg);
		break;

		case MLC_IOCQALPHA:
		retval = mlc_GetTransparencyAlpha(layer->id);
		break;

		case MLC_IOCQFBSIZE:
		if (layer->id > MLC_NUM_LAYERS)
			return -EFAULT;
		retval = fbsize[layer->id];
		break;

		case MLC_IOCTTPCOLOR:
		retval = mlc_SetTransparencyColor(layer->id, arg);
		break;

		case MLC_IOCSPOSITION:
		if(!(_IOC_DIR(cmd) & _IOC_WRITE))
			return -EFAULT;
		if(copy_from_user((void *)&c, argp, 
				  sizeof(struct position_cmd)))
			return -EFAULT;
		retval = mlc_SetPosition(layer->id,
					 c.position.top,
					 c.position.left,
					 c.position.right,
					 c.position.bottom);
		break;

#ifdef CPU_LF1000
		case MLC_IOCSINVISIBLEAREA:
		if(!(_IOC_DIR(cmd) & _IOC_WRITE))
			return -EFAULT;
		if(copy_from_user((void *)&c, argp, 
				  sizeof(struct position_cmd)))
			return -EFAULT;
		retval = mlc_SetLayerInvisibleArea(layer->id,
						   c.position.top,
					  	   c.position.left,
					  	   c.position.right,
					  	   c.position.bottom);
		break;
#endif 

		case MLC_IOCTFORMAT:
		retval = mlc_SetFormat(layer->id, arg);
		break;

		case MLC_IOCTBLEND:
		retval = mlc_SetBlendEnable(layer->id, arg);
		break;

		case MLC_IOCTTRANSP:
		retval = mlc_SetTransparencyEnable(layer->id, arg);
		break;

		case MLC_IOCTINVERT:
		retval = mlc_SetInvertEnable(layer->id, arg);
		break;

		case MLC_IOCTINVCOLOR:
		retval = mlc_SetInvertColor(layer->id, arg);
		break;

		case MLC_IOCTDIRTY:
		retval = mlc_SetDirtyFlag(layer->id);
		break;

		case MLC_IOCQDIRTY:
		retval = mlc_GetDirtyFlag(layer->id);
		break;

		case MLC_IOCGPOSITION:
		if(!(_IOC_DIR(cmd) & _IOC_READ))
			return -EFAULT;
		retval = mlc_GetPosition(layer->id, 
					 (struct mlc_layer_position *)&c);
		if(retval < 0)
			return retval;
		if(copy_to_user(argp, (void *)&c, sizeof(struct position_cmd)))
			return -EFAULT;
		break;

#ifdef CPU_LF1000
		case MLC_IOCGINVISIBLEAREA:
		if(!(_IOC_DIR(cmd) & _IOC_READ))
			return -EFAULT;
		retval = mlc_GetLayerInvisibleArea(layer->id, 
					(struct mlc_layer_position *)&c);
		if(retval < 0)
			return retval;
		if(copy_to_user(argp, (void *)&c, sizeof(struct position_cmd)))
			return -EFAULT;
		break;
#endif 

		case MLC_IOCQHSTRIDE:
		retval = mlc_GetHStride(layer->id);
		break;

		case MLC_IOCQVSTRIDE:
		retval = mlc_GetVStride(layer->id);
		break;

		case MLC_IOCQFORMAT:
		if(mlc_GetFormat(layer->id, &retval) < 0)
			return -EFAULT;
		break;

		case MLC_IOCQADDRESS:
		if(mlc_GetAddress(layer->id, &retval) < 0)
			return -EFAULT;
		break;

		case MLC_IOCSOVERLAYSIZE:
		if(!(_IOC_DIR(cmd) & _IOC_WRITE))
			return -EFAULT;
		if(copy_from_user((void *)&c, argp, 
				  sizeof(struct overlaysize_cmd)))
			return -EFAULT;
		retval = mlc_SetOverlaySize(layer->id,
					    c.overlaysize.srcwidth,
					    c.overlaysize.srcheight,
					    c.overlaysize.dstwidth,
					    c.overlaysize.dstheight);
		break;

		case MLC_IOCGOVERLAYSIZE:
		if(!(_IOC_DIR(cmd) & _IOC_READ))
			return -EFAULT;
		retval = mlc_GetOverlaySize(layer->id, 
					    (struct mlc_overlay_size *)&c);
		if(retval < 0)
			return retval;
		if(copy_to_user(argp, (void *)&c, 
				sizeof(struct overlaysize_cmd)))
			return -EFAULT;
		break;

#ifdef CPU_LF1000
		case MLC_IOCTINVISIBLE:
		retval = mlc_SetLayerInvisibleAreaEnable(layer->id, arg);
		break;
#endif 

#ifdef CPU_LF1000
		case MLC_IOCQINVISIBLE:
		retval = mlc_GetLayerInvisibleAreaEnable(layer->id);
		break;
#endif 

		default:
		return -EFAULT;
	}

	return retval;
}

int mlc_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
			  unsigned long arg)
{
	int retval = 0;
	void __user *argp = (void __user *)arg;
	union mlc_cmd c;

	switch(cmd) {
		case MLC_IOCTENABLE:
		mlc_SetMLCEnable(arg);
		break;

		case MLC_IOCTBACKGND:
		mlc_SetBackground(arg);
		break;

		case MLC_IOCTPRIORITY:
		retval = mlc_SetLayerPriority(arg);
		break;

		case MLC_IOCTTOPDIRTY:
		mlc_SetTopDirtyFlag();
		break;

		case MLC_IOCSSCREENSIZE:
		if(!(_IOC_DIR(cmd) & _IOC_WRITE))
			return -EFAULT;
		if(copy_from_user((void *)&c, argp, sizeof(struct screensize_cmd)))
			return -EFAULT;
		retval = mlc_SetScreenSize(c.screensize.width,
								   c.screensize.height);
		break;

		case MLC_IOCQBACKGND:
		retval = mlc_GetBackground();
		break;

		case MLC_IOCQPRIORITY:
		retval = mlc_GetLayerPriority();
		break;

		case MLC_IOCGSCREENSIZE:
		if(!(_IOC_DIR(cmd) & _IOC_READ))
			return -EFAULT;
		mlc_GetScreenSize((struct mlc_screen_size *)&c);
		if(copy_to_user(argp, (void *)&c, sizeof(struct screensize_cmd)))
			return -EFAULT;
		break;

		default:
		return -ENOTTY;
	}

	return retval;
}

static void mlc_layer_vma_open(struct vm_area_struct *vma)
{
	//printk(KERN_DEBUG "mlc: vma_open virt:%lX, phs %lX\n",
	//		vma->vm_start, vma->vm_pgoff<<PAGE_SHIFT);
}

static void mlc_layer_vma_close(struct vm_area_struct *vma)
{
	//printk(KERN_DEBUG "mlc: vma_close\n");
}

struct vm_operations_struct mlc_layer_vm_ops = {
	.open  = mlc_layer_vma_open,
	.close = mlc_layer_vma_close,
};

static int mlc_layer_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_ops = &mlc_layer_vm_ops;
	vma->vm_flags |= VM_IO;

	ret = io_remap_pfn_range(vma,
				 vma->vm_start, 
				 vma->vm_pgoff, 
				 vma->vm_end - vma->vm_start, 
				 vma->vm_page_prot);
	if(ret < 0) {
		printk(KERN_ALERT "mlc: failed to mmap\n");
		return -EAGAIN;
	}

	mlc_layer_vma_open(vma);
	return 0;
}

static ssize_t mlc_layer_write(struct file *filp, const char __user *buf,
				   size_t count, loff_t *f_pos)
{
	int ret;
	struct mlc_layer *layer;

	layer = container_of(filp->f_dentry->d_inode->i_cdev, 
				 struct mlc_layer, dev);

	if(*f_pos >= fbsize[layer->id])
		return -EFAULT;

	ret = copy_from_user(layer->fb + *f_pos, buf, count);
	if(ret < 0)
		return -EFAULT;

	*f_pos += (count - ret);
	return ret;
}

struct file_operations mlc_fops = {
	.owner = THIS_MODULE,
	.open  = mlc_open,
	.ioctl = mlc_ioctl,
};

struct file_operations mlc_layer_fops = {
	.owner = THIS_MODULE,
	.write = mlc_layer_write,
	.ioctl = mlc_layer_ioctl,
	.mmap  = mlc_layer_mmap,
};

#ifdef CONFIG_PM
static int lf1000_mlc_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int lf1000_mlc_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define lf1000_mlc_suspend	NULL
#define lf1000_mlc_resume	NULL
#endif

static int lf1000_mlc_probe(struct platform_device *pdev)
{
	int ret;
	int i;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);	
	if(!res) {
		printk(KERN_ERR "mlc: failed to get resource\n");
		return -ENXIO;
	}

	if(!request_mem_region(res->start, (res->end - res->start)+1, 
				"MLC" )) {
		printk(KERN_ERR "mlc: failed to request mem region" );
		return -EBUSY;
	}

	mlc.mem = ioremap_nocache(res->start, (res->end - res->start)+1);
	if(mlc.mem == NULL) {
		printk(KERN_ERR "mlc: failed to ioremap\n");
		ret = -ENOMEM;
		goto fail_remap;
	}

	for(i = 0; i < MLC_NUM_LAYERS; i++) {
		mlc.layer[i].fb = ioremap_nocache(fbaddr[i], fbsize[i]);
		if(mlc.layer[i].fb == NULL) {
			ret = -ENOMEM;
			goto fail_remap_fb;
		}
	}
	
	/***********************
 	 * set up the hardware *
 	 ***********************/

	mlc_SetClockMode(PCLKMODE_ONLYWHENCPUACCESS, BCLKMODE_DYNAMIC);
	mlc_SetScreenSize(X_RESOLUTION, Y_RESOLUTION);
	ret = mlc_SetLayerPriority(DISPLAY_VID_LAYER_PRIORITY);
	if(ret < 0)
		printk(KERN_ALERT "mlc: failed to set layer priority %08X\n",
			   DISPLAY_VID_LAYER_PRIORITY);
	mlc_SetFieldEnable(0);

	for(i = 0; i < MLC_NUM_LAYERS; i++) {
		mlc_SetAddress(i, fbaddr[i]);
		init_MUTEX_LOCKED(&mlc.layer[i].sem);
	}

	mlc_SetMLCEnable(1);
	mlc_SetTopDirtyFlag();

	/********************
 	 * register devices *
 	 ********************/

	cdev_init(&mlc.dev, &mlc_fops);
	mlc.dev.owner = THIS_MODULE;
	mlc.dev.ops = &mlc_fops;
	ret = cdev_add(&mlc.dev, MKDEV(MLC_MAJOR,0), 1);
	if(ret < 0) {
		printk(KERN_ERR "mlc: failed to create character device\n");
		goto fail_add_dev;
	}

	for(i = 0; i < MLC_NUM_LAYERS; i++) {
		cdev_init(&(mlc.layer[i].dev), &mlc_layer_fops);
		mlc.layer[i].dev.owner = THIS_MODULE;
		mlc.layer[i].dev.ops = &mlc_layer_fops;
		mlc.layer[i].id = i;
		ret = cdev_add(&mlc.layer[i].dev, MKDEV(MLC_LAYER_MAJOR,i), 1);
		if(ret < 0) {
			printk(KERN_ERR "mlc: failed to create layer dev %d\n",
				i);
			goto fail_add_layers;
		}
	}

#ifdef CONFIG_LF1000_MLC_DEBUG
	sysfs_create_group(&pdev->dev.kobj, &mlc_attr_group);
#endif

	return 0;

fail_add_layers:
	cdev_del(&mlc.dev); /* remove main MLC device */
	for(i -= 1; i >= 0; i--) /* remove layer devices */
		cdev_del(&mlc.layer[i].dev);
	i = MLC_NUM_LAYERS; /* all frame buffers were remapped */
fail_add_dev:
fail_remap_fb:
	iounmap(mlc.mem);
	for(i -= 1; i > 0; i--) {/* release frame buffers that were remapped */
		iounmap(mlc.layer[i].fb);
		release_mem_region(fbaddr[i], fbsize[i]);
	}
fail_remap:
	release_mem_region(res->start, (res->end - res->start) + 1);

	return ret;
}

static int lf1000_mlc_remove(struct platform_device *pdev)
{
	int i;
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);	

#ifdef CONFIG_LF1000_MLC_DEBUG
	sysfs_remove_group(&pdev->dev.kobj, &mlc_attr_group);
#endif
	cdev_del(&mlc.dev);
	for(i = 0; i < MLC_NUM_LAYERS; i++) {
		cdev_del(&(mlc.layer[i].dev));
		iounmap(mlc.layer[i].fb);
		release_mem_region(fbaddr[i], fbsize[i]);
	}

	iounmap(mlc.mem);
	release_mem_region(res->start, (res->end - res->start)+1);

	return 0;
}

static struct platform_driver lf1000_mlc_driver = {
	.probe		= lf1000_mlc_probe,
	.remove		= lf1000_mlc_remove,
	.suspend	= lf1000_mlc_suspend,
	.resume		= lf1000_mlc_resume,
	.driver		= {
		.name   = "lf1000-mlc",
		.owner  = THIS_MODULE,
	},
};

/**************************************
 *  module initialization and cleanup *
 **************************************/

static int __init mlc_init(void)
{
	return platform_driver_register(&lf1000_mlc_driver);
}

static void __exit mlc_cleanup(void)
{
	platform_driver_unregister(&lf1000_mlc_driver);
}

module_init(mlc_init);
module_exit(mlc_cleanup);
MODULE_AUTHOR("Andrey Yurovsky");
MODULE_VERSION("1:0.1");
MODULE_LICENSE("GPL");
