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
#ifndef _LS2K_H
#define _LS2K_H
#include <asm/addrspace.h>
#define LS2K_IO_REG_BASE		0x1f000000

/* CHIP CONFIG regs */
#define LS2K_CHIP_CFG_REG_BASE		(LS2K_IO_REG_BASE + 0x00e10000)
#define LS2K_APBDMA_CONFIG_REG		(LS2K_CHIP_CFG_REG_BASE + 0x438)
#define LS2K_GEN_CONFIG0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x420)
#define LS2K_GEN_CONFIG1_REG		(LS2K_CHIP_CFG_REG_BASE + 0x428)

#define LS2K_PIX0_PLL			(LS2K_CHIP_CFG_REG_BASE + 0x4b0)
#define LS2K_PIX1_PLL			(LS2K_CHIP_CFG_REG_BASE + 0x4c0)

#define LS2K_INT_REG_BASE		(LS2K_CHIP_CFG_REG_BASE + 0x1400)

#define LS2K_INT_ISR0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x1420)
#define LS2K_INT_IEN0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x1424)
#define LS2K_INT_SET0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x1428)
#define LS2K_INT_CLR0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x142c)
#define LS2K_INT_POL0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x1430)
#define LS2K_INT_EDGE0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x1434)
#define LS2K_INT_BOUNCE_REG		(LS2K_CHIP_CFG_REG_BASE + 0x1438)
#define LS2K_INT_AUTO_REG		(LS2K_CHIP_CFG_REG_BASE + 0x143c)

//#define LS2K_GPIO_CFG_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0500)
#define LS2K_GPIO0_OEN_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0500)
#define LS2K_GPIO1_OEN_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0508)
#define LS2K_GPIO0_O_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0510)
#define LS2K_GPIO1_O_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0518)
#define LS2K_GPIO0_I_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0520)
#define LS2K_GPIO1_I_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0528)
#define LS2K_GPIO0_INT_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0530)
#define LS2K_GPIO1_INT_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0538)


#define LS2K_DMA_ORDER_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0c00)
#define LS2K_CHIP_CFG0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0200)
#define LS2K_CHIP_CFG1_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0204)
#define LS2K_CHIP_CFG2_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0208)
#define LS2K_CHIP_CFG3_REG		(LS2K_CHIP_CFG_REG_BASE + 0x020c)
#define LS2K_CHIP_SAMP0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0210)
#define LS2K_CHIP_SAMP1_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0214)
#define LS2K_CHIP_SAMP2_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0218)
#define LS2K_CHIP_SAMP3_REG		(LS2K_CHIP_CFG_REG_BASE + 0x021c)
#define LS2K_CLK_CTRL0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0220)
#define LS2K_CLK_CTRL1_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0224)
#define LS2K_CLK_CTRL2_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0228)
#define LS2K_CLK_CTRL3_REG		(LS2K_CHIP_CFG_REG_BASE + 0x022c)
#define LS2K_PIXCLK0_CTRL0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0230)
#define LS2K_PIXCLK0_CTRL1_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0234)
#define LS2K_PIXCLK1_CTRL0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0238)
#define LS2K_PIXCLK1_CTRL1_REG		(LS2K_CHIP_CFG_REG_BASE + 0x023c)

#define LS2K_WIN_CFG_BASE		(LS2K_CHIP_CFG_REG_BASE + 0x80000)
#define LS2K_M4_WIN0_BASE_REG		(LS2K_WIN_CFG_BASE + 0x0400)
#define LS2K_M4_WIN0_MASK_REG		(LS2K_WIN_CFG_BASE + 0x0440)
#define LS2K_M4_WIN0_MMAP_REG		(LS2K_WIN_CFG_BASE + 0x0480)

/* USB regs */
#define LS2K_EHCI_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00000)
#define LS2K_OHCI_REG_BASE		(LS2K_IO_REG_BASE + 0x00e08000)

/* GMAC regs */
#define LS2K_GMAC0_REG_BASE		(LS2K_IO_REG_BASE + 0x00e10000)
#define LS2K_GMAC1_REG_BASE		(LS2K_IO_REG_BASE + 0x00e18000)

/* HDA regs */
#define LS2K_HDA_REG_BASE		(LS2K_IO_REG_BASE + 0x00e20000)

/* SATAregs */
#define LS2K_SATA_REG_BASE		(LS2K_IO_REG_BASE + 0x00e30000)

/* GPU regs */
#define LS2K_GPU_REG_BASE		(LS2K_IO_REG_BASE + 0x00e40000)

/* DC regs */
#define LS2K_DC_REG_BASE		(LS2K_IO_REG_BASE + 0x00e50000)

#define LS2K_APBDMA_CFG_REG	(LS2K_CHIP_CFG_REG_BASE + 0x0438)
#define LS2K_DMA0_REG		(LS2K_DMA_ORDER_REG + 0x00)
#define LS2K_DMA1_REG		(LS2K_DMA_ORDER_REG + 0x10)
#define LS2K_DMA2_REG		(LS2K_DMA_ORDER_REG + 0x20)
#define LS2K_DMA3_REG		(LS2K_DMA_ORDER_REG + 0x30)
#define LS2K_DMA4_REG		(LS2K_DMA_ORDER_REG + 0x40)


#define LS2K_FB_CFG_DVO0_REG		(0x1240)
#define LS2K_FB_CFG_DVO1_REG		(0x1250)
#define LS2K_FB_ADDR0_DVO0_REG		(0x1260)
#define LS2K_FB_ADDR0_DVO1_REG		(0x1270)
#define LS2K_FB_STRI_DVO0_REG		(0x1280)
#define LS2K_FB_STRI_DVO1_REG		(0x1290)

#define LS2K_FB_DITCFG_DVO0_REG		(0x1360)
#define LS2K_FB_DITCFG_DVO1_REG		(0x1370)
#define LS2K_FB_DITTAB_LO_DVO0_REG	(0x1380)
#define LS2K_FB_DITTAB_LO_DVO1_REG	(0x1390)
#define LS2K_FB_DITTAB_HI_DVO0_REG	(0x13a0)
#define LS2K_FB_DITTAB_HI_DVO1_REG	(0x13b0)
#define LS2K_FB_PANCFG_DVO0_REG		(0x13c0)
#define LS2K_FB_PANCFG_DVO1_REG		(0x13d0)
#define LS2K_FB_PANTIM_DVO0_REG		(0x13e0)
#define LS2K_FB_PANTIM_DVO1_REG		(0x13f0)

#define LS2K_FB_HDISPLAY_DVO0_REG	(0x1400)
#define LS2K_FB_HDISPLAY_DVO1_REG	(0x1410)
#define LS2K_FB_HSYNC_DVO0_REG		(0x1420)
#define LS2K_FB_HSYNC_DVO1_REG		(0x1430)

#define LS2K_FB_VDISPLAY_DVO0_REG	(0x1480)
#define LS2K_FB_VDISPLAY_DVO1_REG	(0x1490)
#define LS2K_FB_VSYNC_DVO0_REG		(0x14a0)
#define LS2K_FB_VSYNC_DVO1_REG		(0x14b0)

#define LS2K_FB_GAMINDEX_DVO0_REG	(0x14e0)
#define LS2K_FB_GAMINDEX_DVO1_REG	(0x14f0)
#define LS2K_FB_GAMDATA_DVO0_REG		(0x1500)
#define LS2K_FB_GAMDATA_DVO1_REG		(0x1510)

#define LS2K_FB_CUR_CFG_REG		(0x1520)
#define LS2K_FB_CUR_ADDR_REG		(0x1530)
#define LS2K_FB_CUR_LOC_ADDR_REG	(0x1540)
#define LS2K_FB_CUR_BACK_REG		(0x1550)
#define LS2K_FB_CUR_FORE_REG		(0x1560)

#define LS2K_FB_INT_REG			(0x1570)

#define LS2K_FB_ADDR1_DVO0_REG		(0x1580)
#define LS2K_FB_ADDR1_DVO1_REG		(0x1590)

#define LS2K_FB_DAC_CTRL_REG		(0x1600)
#define LS2K_FB_DVO_OUTPUT_REG          (0x1630)
/* OTG regs */
#define LS2K_OTG_REG_BASE		(LS2K_IO_REG_BASE + 0x00e60000)

/* SPI regs */
#define LS2K_SPI_REG_BASE		(LS2K_IO_REG_BASE + 0xff0220)

/* UART regs */
#define LS2K_UART0_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00000)
#define LS2K_UART1_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00100)
#define LS2K_UART2_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00200)
#define LS2K_UART3_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00300)
#define LS2K_UART4_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00400)
#define LS2K_UART5_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00500)
#define LS2K_UART6_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00600)
#define LS2K_UART7_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00700)
#define LS2K_UART8_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00800)
#define LS2K_UART9_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00900)
#define LS2K_UART10_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00a00)
#define LS2K_UART11_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00b00)

#define LS2K_CAN0_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00c00)
#define LS2K_CAN1_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00d00)

/* I2C regs */
#define LS2K_I2C0_REG_BASE		(LS2K_IO_REG_BASE + 0x00e01000)
#define LS2K_I2C0_PRER_LO_REG		(LS2K_I2C0_REG_BASE + 0x0)
#define LS2K_I2C0_PRER_HI_REG		(LS2K_I2C0_REG_BASE + 0x1)
#define LS2K_I2C0_CTR_REG   		(LS2K_I2C0_REG_BASE + 0x2)
#define LS2K_I2C0_TXR_REG   		(LS2K_I2C0_REG_BASE + 0x3)
#define LS2K_I2C0_RXR_REG    		(LS2K_I2C0_REG_BASE + 0x3)
#define LS2K_I2C0_CR_REG     		(LS2K_I2C0_REG_BASE + 0x4)
#define LS2K_I2C0_SR_REG     		(LS2K_I2C0_REG_BASE + 0x4)

#define LS2K_I2C1_REG_BASE		(LS2K_IO_REG_BASE + 0x00e01800)
#define LS2K_I2C1_PRER_LO_REG		(LS2K_I2C1_REG_BASE + 0x0)
#define LS2K_I2C1_PRER_HI_REG		(LS2K_I2C1_REG_BASE + 0x1)
#define LS2K_I2C1_CTR_REG    		(LS2K_I2C1_REG_BASE + 0x2)
#define LS2K_I2C1_TXR_REG    		(LS2K_I2C1_REG_BASE + 0x3)
#define LS2K_I2C1_RXR_REG    		(LS2K_I2C1_REG_BASE + 0x3)
#define LS2K_I2C1_CR_REG     		(LS2K_I2C1_REG_BASE + 0x4)
#define LS2K_I2C1_SR_REG     		(LS2K_I2C1_REG_BASE + 0x4)

#define	CR_START			0x80
#define	CR_STOP				0x40
#define	CR_READ				0x20
#define	CR_WRITE			0x10
#define	CR_ACK				0x8
#define	CR_IACK				0x1

#define	SR_NOACK			0x80
#define	SR_BUSY				0x40
#define	SR_AL				0x20
#define	SR_TIP				0x2
#define	SR_IF				0x1

#define LS2K_NAND_DMA_SHIFT		0
#define LS2K_AESR_DMA_SHIFT		3
#define LS2K_AESW_DMA_SHIFT		6
#define LS2K_DESR_DMA_SHIFT		9
#define LS2K_DESW_DMA_SHIFT		12
#define LS2K_SDIO_DMA_SHIFT		15
#define LS2K_I2SS_DMA_SHIFT		18
#define LS2K_I2SR_DMA_SHIFT		21
#define LS2K_AC97O_DMA_SHIFT	24
#define LS2K_AC97I_DMA_SHIFT	27
#define LS2K_APBDMA_MASK		0x7

/* PWM regs */
#define LS2K_PWM_REG_BASE		(LS2K_IO_REG_BASE + 0x00e02000)

/* HPET regs */
#define LS2K_HPET_REG_BASE		(LS2K_IO_REG_BASE + 0x00e04000)

/* AC97 regs */
#define LS2K_AC97_REG_BASE		(LS2K_IO_REG_BASE + 0x00e05000)

/* NAND regs */
#define LS2K_NAND_REG_BASE		(LS2K_IO_REG_BASE + 0x00e06000)
#define LS2K_NAND_CMD_REG		(LS2K_NAND_REG_BASE + 0x0000)
#define LS2K_NAND_ADDR_C_REG		(LS2K_NAND_REG_BASE + 0x0004)
#define LS2K_NAND_ADDR_R_REG		(LS2K_NAND_REG_BASE + 0x0008)
#define LS2K_NAND_TIMING_REG		(LS2K_NAND_REG_BASE + 0x000c)
#define LS2K_NAND_IDL_REG		(LS2K_NAND_REG_BASE + 0x0010)
#define LS2K_NAND_STA_IDH_REG		(LS2K_NAND_REG_BASE + 0x0014)
#define LS2K_NAND_PARAM_REG		(LS2K_NAND_REG_BASE + 0x0018)
#define LS2K_NAND_OP_NUM_REG		(LS2K_NAND_REG_BASE + 0x001c)
#define LS2K_NAND_CSRDY_MAP_REG		(LS2K_NAND_REG_BASE + 0x0020)
#define LS2K_NAND_DMA_ACC_REG		(LS2K_NAND_REG_BASE + 0x0040)

/* ACPI regs */
#define LS2K_ACPI_REG_BASE		(LS2K_IO_REG_BASE + 0x00e07000)
#define LS2K_PM_SOC_REG			(LS2K_ACPI_REG_BASE + 0x0000)
#define LS2K_PM_RESUME_REG		(LS2K_ACPI_REG_BASE + 0x0004)
#define LS2K_PM_RTC_REG			(LS2K_ACPI_REG_BASE + 0x0008)
#define LS2K_PM1_STS_REG		(LS2K_ACPI_REG_BASE + 0x000c)
#define LS2K_PM1_EN_REG			(LS2K_ACPI_REG_BASE + 0x0010)
#define LS2K_PM1_CNT_REG		(LS2K_ACPI_REG_BASE + 0x0014)
#define LS2K_PM1_TMR_REG		(LS2K_ACPI_REG_BASE + 0x0018)
#define LS2K_P_CNT_REG			(LS2K_ACPI_REG_BASE + 0x001c)
#define LS2K_P_LVL2_REG			(LS2K_ACPI_REG_BASE + 0x0020)
#define LS2K_P_LVL3_REG			(LS2K_ACPI_REG_BASE + 0x0024)
#define LS2K_GPE0_STS_REG		(LS2K_ACPI_REG_BASE + 0x0028)
#define LS2K_GPE0_EN_REG		(LS2K_ACPI_REG_BASE + 0x002c)
#define LS2K_RST_CNT_REG		(LS2K_ACPI_REG_BASE + 0x0030)
#define LS2K_WD_SET_REG			(LS2K_ACPI_REG_BASE + 0x0034)
#define LS2K_WD_TIMER_REG		(LS2K_ACPI_REG_BASE + 0x0038)
#define LS2K_DVFS_CNT_REG		(LS2K_ACPI_REG_BASE + 0x003c)
#define LS2K_DVFS_STS_REG		(LS2K_ACPI_REG_BASE + 0x0040)
#define LS2K_MS_CNT_REG			(LS2K_ACPI_REG_BASE + 0x0044)
#define LS2K_MS_THT_REG			(LS2K_ACPI_REG_BASE + 0x0048)
#define	LS2K_THSENS_CNT_REG		(LS2K_ACPI_REG_BASE + 0x004c)
#define LS2K_GEN_RTC1_REG		(LS2K_ACPI_REG_BASE + 0x0050)
#define LS2K_GEN_RTC2_REG		(LS2K_ACPI_REG_BASE + 0x0054)

/* RTC regs */
#define LS2K_RTC_REG_BASE		(LS2K_IO_REG_BASE + 0x00e07800)
#define	LS2K_TOY_TRIM_REG		(LS2K_RTC_REG_BASE + 0x0020)
#define	LS2K_TOY_WRITE0_REG		(LS2K_RTC_REG_BASE + 0x0024)
#define	LS2K_TOY_WRITE1_REG		(LS2K_RTC_REG_BASE + 0x0028)
#define	LS2K_TOY_READ0_REG		(LS2K_RTC_REG_BASE + 0x002c)
#define	LS2K_TOY_READ1_REG		(LS2K_RTC_REG_BASE + 0x0030)
#define	LS2K_TOY_MATCH0_REG		(LS2K_RTC_REG_BASE + 0x0034)
#define	LS2K_TOY_MATCH1_REG		(LS2K_RTC_REG_BASE + 0x0038)
#define	LS2K_TOY_MATCH2_REG		(LS2K_RTC_REG_BASE + 0x003c)
#define	LS2K_RTC_CTRL_REG		(LS2K_RTC_REG_BASE + 0x0040)
#define	LS2K_RTC_TRIM_REG		(LS2K_RTC_REG_BASE + 0x0060)
#define	LS2K_RTC_WRITE0_REG		(LS2K_RTC_REG_BASE + 0x0064)
#define	LS2K_RTC_READ0_REG		(LS2K_RTC_REG_BASE + 0x0068)
#define	LS2K_RTC_MATCH0_REG		(LS2K_RTC_REG_BASE + 0x006c)
#define	LS2K_RTC_MATCH1_REG		(LS2K_RTC_REG_BASE + 0x0070)
#define	LS2K_RTC_MATCH2_REG		(LS2K_RTC_REG_BASE + 0x0074)

#define LS2K_SDIO_REG_BASE		(LS2K_IO_REG_BASE + 0x00e0c000)
#define LS2K_I2S_REG_BASE		(LS2K_IO_REG_BASE + 0x00e0d000)

#define LS2K_INT_MSI_TRIGGER_0		(LS2K_CHIP_CFG_REG_BASE + 0x14b0)
#define LS2K_INT_MSI_TRIGGER_EN_0	(LS2K_CHIP_CFG_REG_BASE + 0x14b4)
#define LS2K_INT_MSI_TRIGGER_1		(LS2K_CHIP_CFG_REG_BASE + 0x14f0)
#define LS2K_INT_MSI_TRIGGER_EN_1	(LS2K_CHIP_CFG_REG_BASE + 0x14f4)
//#define LS2K_IRQ_MASK			0xdf8ff0ffb03ff3ff
#define LS2K_IRQ_MASK			0xff8ff0ffb03ff3ff

/* REG ACCESS*/
#define ls2k_readb(addr)		(*(volatile unsigned char *)TO_UNCAC(addr))
#define ls2k_readw(addr)		(*(volatile unsigned short *)TO_UNCAC(addr))
#define ls2k_readl(addr)		(*(volatile unsigned int *)TO_UNCAC(addr))
#define ls2k_readq(addr)		(*(volatile unsigned long *)TO_UNCAC(addr))
#define ls2k_writeb(val, addr)		*(volatile unsigned char *)TO_UNCAC(addr) = (val)
#define ls2k_writew(val, addr)		*(volatile unsigned short *)TO_UNCAC(addr) = (val)
#define ls2k_writel(val, addr)		*(volatile unsigned int *)TO_UNCAC(addr) = (val)
#define ls2k_writeq(val, addr)		*(volatile unsigned long *)TO_UNCAC(addr) = (val)

/* Board Version Number */
enum {
	LS2K_BOARD_VER_2_2 = 0x4,
	LS2K_BOARD_VER_OLD = 0xf,
};

enum {
	LS3A2H_BOARD_VER_2_2 = 0x4,
	LS3A2H_BOARD_VER_OLD = 0xf,
};


#define LS2K_PCIE_MAX_PORTNUM       3
#define LS2K_PCIE_PORT0             0
#define LS2K_PCIE_PORT1             1
#define LS2K_PCIE_PORT2             2
#define LS2K_PCIE_PORT3             3
#if defined(CONFIG_CPU_LOONGSON3) && defined(CONFIG_LS2K_PCIE_GRAPHIC_CARD)
#define LS2K_PCIE_MEM1_BASE            0x40000000
#define LS2K_PCIE_GET_PORTNUM(sysdata) \
               ((((struct pci_controller *)(sysdata))->mem_resource->start \
                       - LS2K_PCIE_MEM1_BASE) >> 28)
#else
#define LS2K_PCIE_GET_PORTNUM(sysdata) \
               ((((struct pci_controller *)(sysdata))->mem_resource->start \
                       & ~LS2K_PCIE_MEM0_DOWN_MASK) >> 25)
#endif

#define LS2K_CHIP_CFG_REG_CLK_CTRL3     0x22c
#define LS2K_CLK_CTRL3_BIT_PEREF_EN(portnum) (1 << (24 + portnum))

#define LS2K_PCIE_MEM0_BASE_PORT(portnum)	(0x10000000 + (portnum << 25))
#define LS2K_PCIE_IO_BASE_PORT(portnum)		(0x18100000 + (portnum << 22))

#define LS2K_PCIE_REG_BASE_PORT(portnum)        (0x18118000 + (portnum << 22))
#define LS2K_PCIE_PORT_INT_MASK_REG(portnum)	(LS2K_PCIE_REG_BASE_PORT(portnum) + 0x20)
#define LS2K_PCIE_PORT_REG_CTR0			0x0
#define LS2K_PCIE_REG_CTR0_BIT_LTSSM_EN			(1 << 3)
#define LS2K_PCIE_REG_CTR0_BIT_REQ_L1			(1 << 12)
#define LS2K_PCIE_REG_CTR0_BIT_RDY_L23			(1 << 13)
#define LS2K_PCIE_PORT_REG_STAT1		0xC
#define LS2K_PCIE_REG_STAT1_MASK_LTSSM		0x0000003f
#define LS2K_PCIE_REG_STAT1_BIT_LINKUP			(1 << 6)
#define LS2K_PCIE_PORT_REG_CFGADDR		0x24
#define LS2K_PCIE_PORT_REG_CTR_STAT		0x28
#define LS2K_PCIE_REG_CTR_STAT_BIT_ISX4			(1 << 26)
#define LS2K_PCIE_REG_CTR_STAT_BIT_ISRC			(1 << 27)

#define LS2K_PCIE_PORT_HEAD_BASE_PORT(portnum)  (0x18114000 + (portnum << 22))
#define LS2K_PCIE_DEV_HEAD_BASE_PORT(portnum)   (0x18116000 + (portnum << 22))

#define LIE_IN_WINDOW(addr,base,mask)   ((addr & mask) == base)
#define MAP_2_WINDOW(addr,mmap,mask)    ((addr & (~(mask))) | (mmap & mask))
#define LS2K_PCIE_MEM0_DOWN_BASE		0x10000000
#define LS2K_PCIE_MEM0_DOWN_MASK		0xf8000000
#define LS2K_PCIE_MEM0_UP_BASE			0x10000000
#define LS2K_PCIE_MEM0_UP_MASK			0xfe000000
#define LS2K_PCIE_IO_DOWN_BASE			0x18100000
#define LS2K_PCIE_IO_DOWN_MASK			0xff3f0000
#define LS2K_PCIE_IO_UP_BASE			0x0
#define LS2K_PCIE_IO_UP_MASK			0xffff0000

#define LS2K_VER2 2
#define LS2K_VER3 3

#define LS2K_VRAM_2H_DDR 0x01
#define LS2K_VRAM_3A_DDR 0x02
        
#define LS2H_SOC_GPU    0x01
#define LS3A_2H_GPU     0x02
#define LS2K_SOC_GPU    0x03

struct ls2k_usbh_data {
	u8      ports;      /* number of ports on root hub */
	u8      vbus_pin[]; /* port power-control pin */
};

struct generic_plat_data {
	u32	chip_ver;
};

struct ls2k_nand_plat_data {
	int	enable_arbiter;
	u32	nr_parts;
	u32	chip_ver;
	struct mtd_partition *parts;
	int cs;
	u32 csrdy;
};

struct ls2h_gpu_plat_data {
        u32     chip_ver;
        u32     vram_kind; /*gpu vram kind, 0x01: vram in 2H DDR; 0x02: vram in 3A DDR */
        u32     board_kind; /*gpu board kind, 0x01: 2HSOC; 0x02: 3A2H */
};
#endif /* _LS2K_H */
