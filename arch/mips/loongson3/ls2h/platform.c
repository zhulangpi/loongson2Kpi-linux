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
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/resource.h>
#include <linux/serial_8250.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/delay.h>
#include <linux/stmmac.h>
#include <linux/i2c.h>
#include <linux/phy.h>

#include <ls2h/ls2h.h>
#include <ls2h/ls2h_int.h>
#include <linux/i2c-gpio.h>

/*
 * UART
 */
struct plat_serial8250_port ls2h_uart8250_data[] = {
	[0] = {
		.mapbase = CKSEG1ADDR(LS2H_UART0_REG_BASE),	.uartclk = 125000000,
		.membase = (void *)CKSEG1ADDR(LS2H_UART0_REG_BASE),	.irq = LS2H_UART0_IRQ,
		.flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST, 	.iotype = UPIO_MEM,
		.regshift = 0,
	},
	[1] = {
		.mapbase = CKSEG1ADDR(LS2H_UART1_REG_BASE),	.uartclk = 125000000,
		.membase = (void *)CKSEG1ADDR(LS2H_UART1_REG_BASE),	.irq = LS2H_UART1_IRQ,
		.flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST, 	.iotype = UPIO_MEM,
		.regshift = 0,
	},
	[2] = {
		.mapbase = CKSEG1ADDR(LS2H_UART2_REG_BASE),	.uartclk = 125000000,
		.membase = (void *)CKSEG1ADDR(LS2H_UART2_REG_BASE),	.irq = LS2H_UART2_IRQ,
		.flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST, 	.iotype = UPIO_MEM,
		.regshift = 0,
	},
	[3] = {
		.mapbase = CKSEG1ADDR(LS2H_UART3_REG_BASE),	.uartclk = 125000000,
		.membase = (void *)CKSEG1ADDR(LS2H_UART3_REG_BASE),	.irq = LS2H_UART3_IRQ,
		.flags = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST, 	.iotype = UPIO_MEM,
		.regshift = 0,
	},
	{}
};

static struct platform_device uart8250_device = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM1,
	.dev = {
		.platform_data = ls2h_uart8250_data,
	}
};

/*
 * NAND
 */
static struct mtd_partition ls2h_nand_partitions[]={
	[0] = {
		.name   = "kernel",
		.offset = 0,
		.size   = 0x01400000,
	},
	[1] = {
		.name   = "os",
		.offset = 0x01400000,
		.size   = 0x0,

	},
};

static struct ls2h_nand_plat_data ls2h_nand_parts = {
        .enable_arbiter = 1,
        .parts          = ls2h_nand_partitions,
        .nr_parts       = ARRAY_SIZE(ls2h_nand_partitions),
	.chip_ver	= LS2H_VER3,
};

static struct resource ls2h_nand_resources[] = {
	[0] = {
		.start      = 0,
		.end        = 0,
		.flags      = IORESOURCE_DMA,
	},
	[1] = {
		.start      = LS2H_NAND_REG_BASE,
		.end        = LS2H_NAND_REG_BASE + 0x20,
		.flags      = IORESOURCE_MEM,
	},
	[2] = {
		.start      = LS2H_DMA_ORDER_REG,
		.end        = LS2H_DMA_ORDER_REG,
		.flags      = IORESOURCE_MEM,
	},
	[3] = {
		.start      = LS2H_DMA0_IRQ,
		.end        = LS2H_DMA0_IRQ,
		.flags      = IORESOURCE_IRQ,
	},
};

struct platform_device ls2h_nand_device = {
	.name       = "ls2h-nand",
	.id         = 0,
	.dev        = {
		.platform_data = &ls2h_nand_parts,
	},
	.num_resources  = ARRAY_SIZE(ls2h_nand_resources),
	.resource       = ls2h_nand_resources,
};

/*
 * OHCI
 */
static u64 dma_mask = -1;

static struct resource ls2h_ohci_resources[] = {
	[0] = {
		.start = LS2H_OHCI_REG_BASE,
		.end   = (LS2H_OHCI_REG_BASE + 0x1000 - 1),
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = LS2H_OHCI_IRQ,
		.end   = LS2H_OHCI_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct ls2h_usbh_data  ls2h_ohci_platform_data = {
	.ports = 6,
};

static struct platform_device ls2h_ohci_device = {
	.name           = "ls2h-ohci",
	.id             = 0,
	.dev = {
		.platform_data	= &ls2h_ohci_platform_data,
		.dma_mask	= &dma_mask,
	},
	.num_resources  = ARRAY_SIZE(ls2h_ohci_resources),
	.resource       = ls2h_ohci_resources,
};

/*
 * EHCI
 */
static struct resource ls2h_ehci_resources[] = {
	[0] = {
		.start = LS2H_EHCI_REG_BASE,
		.end   = (LS2H_EHCI_REG_BASE + 0x6b),
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = LS2H_EHCI_IRQ,
		.end   = LS2H_EHCI_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct ls2h_usbh_data  ls2h_ehci_platform_data = {
	.ports	= 6,
};

static struct platform_device ls2h_ehci_device = {
	.name	= "ls2h-ehci",
	.id	= 0,
	.dev	= {
		.platform_data	= &ls2h_ehci_platform_data,
		.dma_mask	= &dma_mask,
	},
	.num_resources  = ARRAY_SIZE(ls2h_ehci_resources),
	.resource       = ls2h_ehci_resources,
};

/*
 * GMAC0
 */
static struct stmmac_dma_cfg gmac0_dma_cfg = {
	.pbl =32,
};

static struct stmmac_mdio_bus_data phy0_bus_data = {
	.phy_mask = 0,
};

static struct plat_stmmacenet_data gmac0_plat_dat = {
	.bus_id		= 0,
	.dma_cfg        = &gmac0_dma_cfg,
	.has_gmac	= 1,
	.tx_coe		= 1,
	.enh_desc	= 1,
	.phy_addr	= -1,
	.mdio_bus_data	= &phy0_bus_data,
};

static struct resource ls2h_gmac0_resources[] = {
	[0] = {
		.start = LS2H_GMAC0_REG_BASE,
		.end   = (LS2H_GMAC0_REG_BASE + 0x6b),
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = LS2H_GMAC0_IRQ,
		.end   = LS2H_GMAC0_IRQ,
		.name  = "macirq",
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ls2h_gmac0_device = {
	.name	= "stmmaceth",
	.id	= 0,
	.dev	= {
		.platform_data	= &gmac0_plat_dat,
		.dma_mask	= &dma_mask,
	},
	.num_resources  = ARRAY_SIZE(ls2h_gmac0_resources),
	.resource       = ls2h_gmac0_resources,
};

/*
 * GMAC1
 */

static struct stmmac_dma_cfg gmac1_dma_cfg = {
	.pbl =32,
};

static struct stmmac_mdio_bus_data phy1_bus_data = {
	.phy_mask = 0,
};

static struct plat_stmmacenet_data gmac1_plat_dat = {
	.bus_id		= 1,
	.dma_cfg	= &gmac1_dma_cfg,
	.has_gmac	= 1,
	.tx_coe		= 1,
	.enh_desc	= 1,
	.mdio_bus_data  = &phy1_bus_data ,
	.phy_addr	= -1,
};

static struct resource ls2h_gmac1_resources[] = {
	[0] = {
		.start = LS2H_GMAC1_REG_BASE,
		.end   = (LS2H_GMAC1_REG_BASE + 0x6b),
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = LS2H_GMAC1_IRQ,
		.end   = LS2H_GMAC1_IRQ,
		.name  = "macirq",
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ls2h_gmac1_device = {
	.name           = "stmmaceth",
	.id             = 1,
	.dev = {
		.platform_data = &gmac1_plat_dat,
		.dma_mask = &dma_mask,
	},
	.num_resources  = ARRAY_SIZE(ls2h_gmac1_resources),
	.resource       = ls2h_gmac1_resources,
};

/*
 * AHCI
 */
static struct resource ls2h_ahci_resources[] = {
	[0] = {
		.start = LS2H_SATA_REG_BASE,
		.end   = LS2H_SATA_REG_BASE + 0x1ff,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = LS2H_SATA_IRQ,
		.end   = LS2H_SATA_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ls2h_ahci_device = {
	.name           = "ahci",
	.id             = 0,
	.dev = {
		.dma_mask	= &dma_mask,
	},
	.num_resources  = ARRAY_SIZE(ls2h_ahci_resources),
	.resource       = ls2h_ahci_resources,
};

/*
 * OTG
 */
#if     !defined(CONFIG_CPU_LOONGSON3) || !defined(CONFIG_SUSPEND)
static struct resource ls2h_otg_resources[] = {
	[0] = {
		.start = LS2H_OTG_REG_BASE,
		.end   = (LS2H_OTG_REG_BASE + 0xfffff),
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = LS2H_OTG_IRQ,
		.end   = LS2H_OTG_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ls2h_otg_device = {
	.name           = "dwc_otg",
	.id             = 0,
	.dev = {
		.dma_mask = &dma_mask,
	},
	.num_resources  = ARRAY_SIZE(ls2h_otg_resources),
	.resource       = ls2h_otg_resources,
};
#endif
/*
 * RTC
 */
static struct resource ls2h_rtc_resources[] = {
       [0] = {
               .start  = LS2H_RTC_REG_BASE,
               .end    = (LS2H_RTC_REG_BASE + 0xff),
               .flags  = IORESOURCE_MEM,
       },
       [1] = {
               .start  = LS2H_RTC_INT0_IRQ,
               .end    = LS2H_TOY_TICK_IRQ,
               .flags  = IORESOURCE_IRQ,
       },
};

static struct platform_device ls2h_rtc_device = {
       .name   = "ls2h-rtc",
       .id     = 0,
       .num_resources  = ARRAY_SIZE(ls2h_rtc_resources),
       .resource       = ls2h_rtc_resources,
};


/*
 * DC
 */
static struct resource ls2h_dc_resources[] = {
	[0] = {
		.start	= LS2H_DC_REG_BASE,
		.end	= LS2H_DC_REG_BASE + 0x2000,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= LS2H_DC_IRQ,
		.end	= LS2H_DC_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device ls2h_dc_device = {
	.name           = "ls2h-fb",
	.id             = 0,
	.num_resources	= ARRAY_SIZE(ls2h_dc_resources),
	.resource	= ls2h_dc_resources,
};

/*
 * HD Audio
 */
static struct generic_plat_data ls2h_hda_data = {
	.chip_ver = LS2H_VER3,
};

static struct resource ls2h_audio_resources[] = {
	[0] = {
		.start = LS2H_HDA_REG_BASE,
		.end   = LS2H_HDA_REG_BASE + 0x17f,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = LS2H_HDA_IRQ,
		.end   = LS2H_HDA_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ls2h_audio_device = {
	.name           = "ls2h-audio",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(ls2h_audio_resources),
	.resource       = ls2h_audio_resources,
	.dev		= {
		.platform_data = &ls2h_hda_data,
	}
};

/*
 * I2C
 */
static struct resource ls2h_i2c0_resources[] = {
	[0] = {
		.start = LS2H_I2C0_REG_BASE,
		.end   = LS2H_I2C0_REG_BASE + 0x8,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = LS2H_I2C0_IRQ,
		.end   = LS2H_I2C0_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ls2h_i2c0_device = {
	.name           = "ls2h-i2c",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(ls2h_i2c0_resources),
	.resource       = ls2h_i2c0_resources,
};

static struct resource ls2h_i2c1_resources[] = {
	[0] = {
		.start = LS2H_I2C1_REG_BASE,
		.end   = LS2H_I2C1_REG_BASE + 0x8,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = LS2H_I2C1_IRQ,
		.end   = LS2H_I2C1_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ls2h_i2c1_device = {
	.name           = "ls2h-i2c",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(ls2h_i2c1_resources),
	.resource       = ls2h_i2c1_resources,
};

/*
 * GPU
 */
static struct ls2h_gpu_plat_data ls2h_gpu_data = {
	.chip_ver = LS2H_VER3,
	.vram_kind = LS2H_VRAM_2H_DDR,
	.board_kind = LS3A_2H_GPU,
};

static struct resource ls2h_gpu_resources[] = {
	[0] = {
		.name	= "gpu_base",
		.start	= LS2H_GPU_REG_BASE,
		.end	= LS2H_GPU_REG_BASE + 0x7ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.name	= "gpu_irq",
		.start	= LS2H_GPU_IRQ,
		.end	= LS2H_GPU_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.name	= "gpu_mem",
		.start	= 0xe0004000000,
		.end	= 0xe000bffffff,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ls2h_gpu_device = {
	.name           = "galcore",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(ls2h_gpu_resources),
	.resource       = ls2h_gpu_resources,
	.dev		= {
		.platform_data = &ls2h_gpu_data,
	}
};
/*
 * GPIO-I2C
 */
static struct i2c_gpio_platform_data pdata = {
	.sda_pin                = LS2H_GPIO_PIN_5,
	.sda_is_open_drain      = 0,
	.scl_pin                = LS2H_GPIO_PIN_4,
	.scl_is_open_drain      = 0,
	.udelay                 = 100,
};

static struct platform_device ls2h_gpio_i2c_device = {
	.name                   = "i2c-gpio",
	.id                     = 2,
	.num_resources          = 0,
	.resource               = NULL,
	.dev                    = {
		.platform_data  = &pdata,
	},
};

static struct platform_device *ls2h_platform_devices[] = {
	&uart8250_device,
	&ls2h_i2c0_device,
	&ls2h_i2c1_device,
	&ls2h_nand_device,
	&ls2h_ohci_device,
	&ls2h_ehci_device,
	&ls2h_gmac0_device,
	&ls2h_gmac1_device,
	&ls2h_ahci_device,
	&ls2h_dc_device,
	&ls2h_audio_device,
#if     !defined(CONFIG_CPU_LOONGSON3) || !defined(CONFIG_SUSPEND)
	&ls2h_otg_device,
#endif
	&ls2h_rtc_device,
	&ls2h_gpu_device,
};

static struct platform_device *ls2h_i2c_gpio_platform_devices[] = {
	&ls2h_gpio_i2c_device,
};

const struct i2c_board_info __initdata ls2h_gmac_eep_info = {
	I2C_BOARD_INFO("eeprom-mac", 0x50),
};

const struct i2c_board_info __initdata ls2h_fb_eep_info = {
	I2C_BOARD_INFO("eeprom-edid", 0x50),
};

const struct i2c_board_info __initdata ls2h_dvi_fb_eep_info = {
	I2C_BOARD_INFO("dvi-eeprom-edid", 0x50),
};

int ls2h_platform_init(void)
{
	int tmp;

	/*
	 * chip_config0 : 0x1fd00200
	 *
	 * bit[26]:	usb reset,		0: reset        1: disable reset
	 * bit[14]:	host/otg select,	0: host         1: otg
	 * bit[4]:	ac97/hda select,	0: ac97		1: hda
	 *
	 */

	tmp = ls2h_readl(LS2H_CHIP_CFG0_REG);
	ls2h_writel(tmp | (1 << 26) | (1 << 14) | (1 << 4), LS2H_CHIP_CFG0_REG);

#define I2C_BUS_0 0
#define I2C_BUS_1 1
	tmp = ls2h_readl(LS2H_GPIO_IN_REG);
	tmp = (tmp >> 8) & 0xf;

	if ((read_c0_prid() & 0xf) == PRID_REV_LOONGSON3A){
		if (tmp == LS3A2H_BOARD_VER_OLD)
			i2c_register_board_info(I2C_BUS_1, &ls2h_gmac_eep_info, 1);
		else
			i2c_register_board_info(I2C_BUS_0, &ls2h_gmac_eep_info, 1);
	}else{
		i2c_register_board_info(I2C_BUS_0, &ls2h_gmac_eep_info, 1);
	}
	i2c_register_board_info(I2C_BUS_1, &ls2h_fb_eep_info, 1);

	if ((read_c0_prid() & 0xf) != PRID_REV_LOONGSON3A){
		i2c_register_board_info(2, &ls2h_dvi_fb_eep_info, 1);
		/* gpio 4 5 used for simulation an i2c device.gpio 4 -> SDA,gpio 5 ->SCL */
		ls2h_readl(LS2H_GPIO_CFG_REG) &= (~((1 << LS2H_GPIO_PIN_4) || (1 << LS2H_GPIO_PIN_5)));
	}

	/* This register is zero in 2h3, !zero in 2h2 */
	tmp = ls2h_readl(LS2H_CHIP_SAMP3_REG);
	if (tmp) {
		ls2h_gpu_data.chip_ver = LS2H_VER2;
		ls2h_hda_data.chip_ver = LS2H_VER2;
		ls2h_nand_parts.chip_ver = LS2H_VER2;
	}
	pr_info("South-Bridge 2H Chipe Version: %s\n", tmp? "VER2" : "VER3");

	tmp = uma_vram_size;
	if(tmp) {
		ls2h_gpu_data.vram_kind = LS2H_VRAM_3A_DDR;	
		ls2h_gpu_resources[2].start = uma_vram_addr;
		ls2h_gpu_resources[2].end = uma_vram_addr + (uma_vram_size << 20) - 1;
	}
	pr_info("GPU USE : %s DDR as VRAM\n", tmp? "3A " : "2H");
	pr_info("VRAM size is :0x%lx \n", uma_vram_size << 20);
	if ((read_c0_prid() & 0xf) != PRID_REV_LOONGSON3A){
		platform_add_devices(ls2h_i2c_gpio_platform_devices,
			ARRAY_SIZE(ls2h_i2c_gpio_platform_devices));
	}
	return platform_add_devices(ls2h_platform_devices,
			ARRAY_SIZE(ls2h_platform_devices));
}
