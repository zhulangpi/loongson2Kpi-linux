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
#include <ls2k.h>
#include <ls2k_int.h>
extern struct pci_ops ls2k_pcie_ops_port0;

/* PCI-E controller0 X4 can't use whole memory space */
static struct resource ls2k_pcie_mem_resource0 = {
	.name	= "LS2H PCIE0 MEM",
	.start	= 0x40000000UL,
	.end	= 0x6fffffffUL,
	.flags	= IORESOURCE_MEM,
};

static struct resource ls2k_pcie_io_resource0 = {
	.name	= "LS2H PCIE0 IO MEM",
	.start	= 0x00004000UL,
	.end	= 0x1ffffffUL,
	.flags	= IORESOURCE_IO,
};

static struct pci_controller ls2k_pcie_controller0 = {
	.pci_ops	= &ls2k_pcie_ops_port0,
	.io_resource	= &ls2k_pcie_io_resource0,
	.mem_resource	= &ls2k_pcie_mem_resource0,
	.mem_offset	= 0x00000000UL,
	.io_offset	= 0x00000000UL,
        .io_map_base    = 0xffffffffb8000000UL,
};

#ifdef CONFIG_DISABLE_PCI
static int disablepci = 1;
#else
static int disablepci = 0;
#endif

int __init ls2k_pcie_init(void)
{
	ioport_resource.end = 0xffffffff;

	if (disablepci)
		return 0;

	pr_info("arch_initcall:pcibios_init\n");

	register_pci_controller(&ls2k_pcie_controller0);


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


__setup("disablepci=", disablepci_setup);

arch_initcall(ls2k_pcie_init);
