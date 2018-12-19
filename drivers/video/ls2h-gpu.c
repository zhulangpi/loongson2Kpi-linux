/****************************************************************************
*
*    Copyright (C) 2005 - 2013 by Vivante Corp.
*
*    This program is free software; you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation; either version 2 of the license, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program; if not write to the Free Software
*    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*
*****************************************************************************/

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/platform_device.h>

#define DEVICE_NAME "galcore"

int __init ls2hgpu_drv_open(struct inode *inode, struct file *filp);

int __init ls2hgpu_drv_release(struct inode *inode, struct file *filp);

long __init ls2hgpu_drv_ioctl(struct file *filp,
		       unsigned int ioctlCode, unsigned long arg);

int __init ls2hgpu_drv_mmap(struct file *filp, struct vm_area_struct *vma);

int  __init ls2hgpu_gpu_probe(struct platform_device *pdev);

int __init ls2hgpu_gpu_remove(struct platform_device *pdev);

int __init ls2hgpu_gpu_suspend(struct platform_device *dev,
				  pm_message_t state);

int __init ls2hgpu_gpu_resume(struct platform_device *dev);

struct file_operations driver_fops = {
	.owner		= THIS_MODULE,
	.open		= ls2hgpu_drv_open,
	.release	= ls2hgpu_drv_release,
	.unlocked_ioctl	= ls2hgpu_drv_ioctl,
#ifdef HAVE_COMPAT_IOCTL
	.compat_ioctl	= ls2hgpu_drv_ioctl,
#endif
	.mmap		= ls2hgpu_drv_mmap,
};

static struct platform_driver gpu_driver = {
	.probe		= ls2hgpu_gpu_probe,
	.remove		= ls2hgpu_gpu_remove,

	.suspend	= ls2hgpu_gpu_suspend,
	.resume		= ls2hgpu_gpu_resume,

	.driver		= {
		   .name = DEVICE_NAME,
	}
};

static int __init gpu_init(void)
{
	return platform_driver_register(&gpu_driver);
}

static void __exit gpu_exit(void)
{
	platform_driver_unregister(&gpu_driver);
}

module_init(gpu_init);
module_exit(gpu_exit);
#ifdef MODULE
MODULE_DESCRIPTION("Loongson2H Graphics Driver");
MODULE_LICENSE("GPL");
#endif
