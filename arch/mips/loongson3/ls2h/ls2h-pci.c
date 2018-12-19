/*
 *  Copyright (C) 2013, Loongson Technology Corporation Limited, Inc.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 */
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <ls2h/ls2h.h>
#include <ls2h/ls2h_int.h>
extern struct pci_ops ls2h_pcie_ops_port0;
extern struct pci_ops ls2h_pcie_ops_port1;
extern struct pci_ops ls2h_pcie_ops_port2;
extern struct pci_ops ls2h_pcie_ops_port3;
extern int ls2h_pcie_bios_init(struct pci_dev *dev);

#ifdef CONFIG_SWIOTLB
extern struct mips_dma_map_ops ls2h_pcie_dma_map_ops;
#endif

/* PCI-E controller0 X4 can't use whole memory space */
static struct resource ls2h_pcie_mem_resource0 = {
	.name	= "LS2H PCIE0 MEM",
	.start	= 0x40000000UL,
	.end	= 0x4fffffffUL,
	.flags	= IORESOURCE_MEM,
};

static struct resource ls2h_pcie_io_resource0 = {
	.name	= "LS2H PCIE0 IO MEM",
	.start	= 0x18100000UL,
	.end	= 0x1810ffffUL,
	.flags	= IORESOURCE_IO,
};

static struct pci_controller ls2h_pcie_controller0 = {
	.pci_ops	= &ls2h_pcie_ops_port0,
	.io_resource	= &ls2h_pcie_io_resource0,
	.mem_resource	= &ls2h_pcie_mem_resource0,
	.mem_offset	= 0x00000000UL,
	.io_offset	= 0x00000000UL,
#if defined(CONFIG_CPU_LOONGSON3) && defined(CONFIG_LS2H_PCIE_GRAPHIC_CARD)
        .io_map_base    = 0x18100000UL,
#else
        .io_map_base    = 0x00000000UL,
#endif
};

/* PCI-E controller1 */
static struct resource ls2h_pcie_mem_resource1 = {
	.name	= "LS2H PCIE1 MEM",
#if defined(CONFIG_CPU_LOONGSON3) && defined(CONFIG_LS2H_PCIE_GRAPHIC_CARD)
	.start  = 0x50000000UL,
	.end    = 0x5fffffffUL,
#else
	.start	= 0x12000000UL,
	.end	= 0x13ffffffUL,
#endif
	.flags	= IORESOURCE_MEM,
};

static struct resource ls2h_pcie_io_resource1 = {
	.name	= "LS2H PCIE1 IO MEM",
	.start	= 0x18500000UL,
	.end	= 0x1850ffffUL,
	.flags	= IORESOURCE_IO,
};

static struct pci_controller ls2h_pcie_controller1 = {
	.pci_ops	= &ls2h_pcie_ops_port1,
	.io_resource	= &ls2h_pcie_io_resource1,
	.mem_resource	= &ls2h_pcie_mem_resource1,
	.mem_offset	= 0x00000000UL,
	.io_offset	= 0x00000000UL,
	.io_map_base	= 0x00000000UL,
};

/* PCI-E controller2 */
static struct resource ls2h_pcie_mem_resource2 = {
	.name	= "LS2H PCIE2 MEM",
#if defined(CONFIG_CPU_LOONGSON3) && defined(CONFIG_LS2H_PCIE_GRAPHIC_CARD)
	.start  = 0x60000000UL,
	.end    = 0x6fffffffUL,
#else
	.start	= 0x14000000UL,
	.end	= 0x15ffffffUL,
#endif
	.flags	= IORESOURCE_MEM,
};

static struct resource ls2h_pcie_io_resource2 = {
	.name	= "LS2H PCIE2 IO MEM",
	.start	= 0x18900000UL,
	.end	= 0x1890ffffUL,
	.flags	= IORESOURCE_IO,
};

static struct pci_controller ls2h_pcie_controller2 = {
	.pci_ops	= &ls2h_pcie_ops_port2,
	.io_resource	= &ls2h_pcie_io_resource2,
	.mem_resource	= &ls2h_pcie_mem_resource2,
	.mem_offset	= 0x00000000UL,
	.io_offset	= 0x00000000UL,
	.io_map_base	= 0x00000000UL,
};

/* PCI-E controller3 */
static struct resource ls2h_pcie_mem_resource3 = {
	.name	= "LS2H PCIE3 MEM",
#if defined(CONFIG_CPU_LOONGSON3) && defined(CONFIG_LS2H_PCIE_GRAPHIC_CARD)
	.start  = 0x70000000UL,
	.end    = 0x7fffffffUL,
#else
	.start	= 0x16000000UL,
	.end	= 0x17ffffffUL,
#endif
	.flags	= IORESOURCE_MEM,
};

static struct resource ls2h_pcie_io_resource3 = {
	.name	= "LS2H PCIE3 IO MEM",
	.start	= 0x18d00000UL,
	.end	= 0x18d0ffffUL,
	.flags	= IORESOURCE_IO,
};

static struct pci_controller ls2h_pcie_controller3 = {
	.pci_ops	= &ls2h_pcie_ops_port3,
	.io_resource	= &ls2h_pcie_io_resource3,
	.mem_resource	= &ls2h_pcie_mem_resource3,
	.mem_offset	= 0x00000000UL,
	.io_offset	= 0x00000000UL,
	.io_map_base	= 0x00000000UL,
};

#ifdef CONFIG_DISABLE_PCI
static int disablepci = 1;
#else
static int disablepci = 0;
#endif

static void en_ref_clock(void)
{
	unsigned int data;

	data = ls2h_readl(LS2H_CLK_CTRL3_REG);
	data |= (LS2H_CLK_CTRL3_BIT_PEREF_EN(0)
		 | LS2H_CLK_CTRL3_BIT_PEREF_EN(1)
		 | LS2H_CLK_CTRL3_BIT_PEREF_EN(2)
		 | LS2H_CLK_CTRL3_BIT_PEREF_EN(3));
	ls2h_writel(data, LS2H_CLK_CTRL3_REG);
}

static int is_rc_mode(void)
{
	unsigned data;

	data = ls2h_readl(LS2H_PCIE_REG_BASE_PORT(0)
			| LS2H_PCIE_PORT_REG_CTR_STAT);

	return data & LS2H_PCIE_REG_CTR_STAT_BIT_ISRC;
}

static int is_x4_mode(void)
{
	unsigned data;

	data = ls2h_readl(LS2H_PCIE_REG_BASE_PORT(0)
			| LS2H_PCIE_PORT_REG_CTR_STAT);

	return data & LS2H_PCIE_REG_CTR_STAT_BIT_ISX4;
}

void ls2h_pcie_port_init(int port)
{
	unsigned reg, data;

	reg = LS2H_PCIE_PORT_HEAD_BASE_PORT(port) | 0x7c;
	data = ls2h_readl(reg) & (~0xf);
	data |=1;
	ls2h_writel(data, reg);

	reg = LS2H_PCIE_REG_BASE_PORT(port) | LS2H_PCIE_PORT_REG_CTR0;
	ls2h_writel(0xff204c, reg);

	reg = LS2H_PCIE_PORT_HEAD_BASE_PORT(port) | PCI_CLASS_REVISION;
	data = ls2h_readl(reg) & 0xffff;
	data |= (PCI_CLASS_BRIDGE_PCI << 16);
	ls2h_writel(data, reg);
}

int ls2h_init_pcie_bios(struct pci_dev *pdev)
{
#ifdef CONFIG_SWIOTLB
	pdev->dev.archdata.dma_ops = &ls2h_pcie_dma_map_ops;
#endif
	return ls2h_pcie_bios_init(pdev);
}

int __init ls2h_pcie_init(void)
{
	ioport_resource.end = 0xffffffff;

	if (disablepci)
		return 0;

	pr_info("arch_initcall:pcibios_init\n");
	en_ref_clock();

	if (!is_rc_mode())
		return 0;

#if defined(CONFIG_CPU_LOONGSON3) && defined(CONFIG_LS2H_PCIE_GRAPHIC_CARD)
	if (is_x4_mode())
		ls2h_pcie_mem_resource0.end = 0x7fffffffUL;
#endif

	ls2h_pcie_port_init(0);
	register_pci_controller(&ls2h_pcie_controller0);

	if (is_x4_mode())
		return 0;

	ls2h_pcie_port_init(1);
	register_pci_controller(&ls2h_pcie_controller1);

	ls2h_pcie_port_init(2);
	register_pci_controller(&ls2h_pcie_controller2);

	ls2h_pcie_port_init(3);
	register_pci_controller(&ls2h_pcie_controller3);

	return 0;
}

static int __init disablepci_setup(char *options)
{
	if (!options || !*options)
		return 0;
	if (options[0] == '0')
		disablepci = 0;
	else
		disablepci = simple_strtoul(options, 0, 0);
	return 1;
}

#if     defined(CONFIG_CPU_LOONGSON3)&&defined(CONFIG_SUSPEND)
void ls2h_early_config(void)
{
	u32 val;

	/*  
	 *           * Loongson-2H chip_config0: 0x1fd00200
	 *           * bit[5]:      Loongson-2H bridge mode,0: disable      1: enable
	 *           * bit[4]:      ac97/hda select,        0: ac97         1: hda
	 *           * bit[14]:     host/otg select,        0: host         1: otg
	 *           * bit[26]:     usb reset,              0: enable       1: disable
	 **/

	val = ls2h_readl(LS2H_CHIP_CFG0_REG);
	ls2h_writel(val | (1 << 5) | (1 << 4) | (1 << 14) | (1 << 26), LS2H_CHIP_CFG0_REG);

	en_ref_clock();

	if (is_rc_mode()) {
		ls2h_pcie_port_init(0);
		if (!is_x4_mode()) {
			ls2h_pcie_port_init(1);
			ls2h_pcie_port_init(2);
			ls2h_pcie_port_init(3);
		}   
	}
}
#endif

__setup("disablepci=", disablepci_setup);
