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
#include <linux/module.h>
#include <linux/pci.h>
#ifdef CONFIG_MACH_LOONGSON2K
#include <ls2k.h>
#define LS2H_VER3 LS2K_VER3
#define LS2H_VRAM_2H_DDR LS2K_VRAM_2H_DDR
#endif

#define DEVICE_NAME "galcore"

extern int ls2hgpu_gpu_probe(struct platform_device *pdev);

extern int ls2hgpu_gpu_remove(struct platform_device *pdev);

extern int ls2hgpu_gpu_suspend(struct platform_device *dev,
	       	  pm_message_t state);

extern int ls2hgpu_gpu_resume(struct platform_device *dev);

static struct platform_driver gpu_driver = {
	.probe		= ls2hgpu_gpu_probe,
	.remove		= ls2hgpu_gpu_remove,
	.suspend	= ls2hgpu_gpu_suspend,
	.resume		= ls2hgpu_gpu_resume,
	.driver		= {
		   .name = DEVICE_NAME,
	}
};


static struct pci_device_id ls2k_gpu_devices[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_LOONGSON, PCI_DEVICE_ID_LOONGSON_GPU)},
	{0, 0, 0, 0, 0, 0, 0}
};

/*
 * GPU
 */
static struct ls2h_gpu_plat_data ls2k_gpu_data = {
	.chip_ver = LS2H_VER3,
	.vram_kind = LS2H_VRAM_2H_DDR,
	.board_kind = LS2K_SOC_GPU,
};

static struct resource ls2k_gpu_resources[] = {
	[0] = {
		.name	= "gpu_base",
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.name	= "gpu_irq",
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.name	= "gpu_mem",
		.start	= 0x0000a000000,
		.end	= 0x0000dffffff,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ls2k_gpu_device = {
	.name           = "galcore",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(ls2k_gpu_resources),
	.resource       = ls2k_gpu_resources,
	.dev		= {
		.platform_data = &ls2k_gpu_data,
	}
};


static int ls2k_gpu_pci_register(struct pci_dev *pdev,
				 const struct pci_device_id *ent)
{
	int ret;

	pr_debug("ls2k_gpu_pci_register BEGIN\n");
	
	/* Enable device in PCI config */
	ret = pci_enable_device(pdev);
	if (ret < 0) {
		printk(KERN_ERR "ls2kfb (%s): Cannot enable PCI device\n",
		       pci_name(pdev));
		goto err_out;
	}

	/* request the mem regions */
	ret = pci_request_region(pdev, 0, "ls2kfb io");
	if (ret < 0) {
		printk( KERN_ERR "ls2kfb (%s): cannot request region 0.\n",
			pci_name(pdev));
		goto err_out;
	}

	ls2k_gpu_resources[0].start = pci_resource_start (pdev, 0);
	ls2k_gpu_resources[0].end = pci_resource_end(pdev, 0);
	ls2k_gpu_resources[1].start = pdev->irq;
	ls2k_gpu_resources[1].end = pdev->irq;


	platform_device_register(&ls2k_gpu_device);


	return 0;
err_out:
	return ret;
}

static void ls2k_gpu_pci_unregister(struct pci_dev *pdev)
{

	platform_device_unregister(&ls2k_gpu_device);
	pci_release_region(pdev, 0);
}

int ls2k_gpu_pci_suspend(struct pci_dev *pdev, pm_message_t mesg)
{
	pci_save_state(pdev);
	return 0;
}

int ls2k_gpu_pci_resume(struct pci_dev *pdev)
{
	return 0;
}


static struct pci_driver ls2k_gpu_pci_driver = {
	.name		= "ls2k-gpu",
	.id_table	= ls2k_gpu_devices,
	.probe		= ls2k_gpu_pci_register,
	.remove		= ls2k_gpu_pci_unregister,
#ifdef	CONFIG_SUSPEND
	.suspend = ls2k_gpu_pci_suspend,
	.resume	 = ls2k_gpu_pci_resume,
#endif
};

static int __init gpu_init(void)
{
	int ret;
	
	ret = platform_driver_register(&gpu_driver);
	if(ret) return ret;
	ret = pci_register_driver (&ls2k_gpu_pci_driver);
	return ret;
}

static void __exit gpu_exit(void)
{
	platform_driver_unregister(&gpu_driver);
	pci_unregister_driver (&ls2k_gpu_pci_driver);
}

module_init(gpu_init);
module_exit(gpu_exit);
MODULE_DESCRIPTION("Loongson2H Graphics Driver");
MODULE_LICENSE("GPL");
