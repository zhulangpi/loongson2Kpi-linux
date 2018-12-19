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
#ifndef _LS2K_INT_H
#define _LS2K_INT_H

#define LS2K_IRQ_OFF 			64

#define LS2K_IRQ_BASE			LS2K_IRQ_OFF

/* group 0 */
#define LS2K_UART0_IRQ			(0 + LS2K_IRQ_OFF)
#define LS2K_UART4_IRQ			(1 + LS2K_IRQ_OFF)
#define LS2K_UART8_IRQ			(2 + LS2K_IRQ_OFF)
#define LS2K_E1_IRQ			(3 + LS2K_IRQ_OFF)
#define LS2K_HDA_IRQ			(4 + LS2K_IRQ_OFF)
#define LS2K_I2S_IRQ			(5 + LS2K_IRQ_OFF)
#define LS2K_AC97_IRQ			(6 + LS2K_IRQ_OFF)
#define LS2K_THSENSE_IRQ		(7 + LS2K_IRQ_OFF)
#define LS2K_TOY_IRQ			(8 + LS2K_IRQ_OFF)
#define LS2K_RTC_IRQ			(9 + LS2K_IRQ_OFF)
#define LS2K_GMAC0_IRQ			(12 + LS2K_IRQ_OFF)
#define LS2K_GMAC0_PMT_IRQ		(13 + LS2K_IRQ_OFF)
#define LS2K_GMAC1_IRQ			(14 + LS2K_IRQ_OFF)
#define LS2K_GMAC1_PMT_IRQ		(15 + LS2K_IRQ_OFF)
#define LS2K_CAN0_IRQ			(16 + LS2K_IRQ_OFF)
#define LS2K_CAN1_IRQ			(17 + LS2K_IRQ_OFF)
#define LS2K_BOOT_IRQ			(18 + LS2K_IRQ_OFF)
#define LS2K_SATA_IRQ			(19 + LS2K_IRQ_OFF)
#define LS2K_NAND_IRQ			(20 + LS2K_IRQ_OFF)
#define LS2K_HPET_IRQ			(21 + LS2K_IRQ_OFF)
#define LS2K_I2C0_IRQ			(22 + LS2K_IRQ_OFF)
#define LS2K_I2C1_IRQ			(23 + LS2K_IRQ_OFF)
#define LS2K_PWM0_IRQ			(24 + LS2K_IRQ_OFF)
#define LS2K_PWM1_IRQ			(25 + LS2K_IRQ_OFF)
#define LS2K_PWM2_IRQ			(26 + LS2K_IRQ_OFF)
#define LS2K_PWM3_IRQ			(27 + LS2K_IRQ_OFF)
#define LS2K_DC_IRQ			(28 + LS2K_IRQ_OFF)
#define LS2K_GPU_IRQ			(29 + LS2K_IRQ_OFF)
#define LS2K_SDIO_IRQ			(31 + LS2K_IRQ_OFF)

/* group 1 */
#define LS2K_PCIE_PORT0_INTA_IRQ	(32 + LS2K_IRQ_OFF)
#define LS2K_PCIE_PORT1_INTA_IRQ	(36 + LS2K_IRQ_OFF)

#define LS2K_TOY_INT0_IRQ		(40 + LS2K_IRQ_OFF)
#define LS2K_TOY_INT1_IRQ		(41 + LS2K_IRQ_OFF)
#define LS2K_TOY_INT2_IRQ		(42 + LS2K_IRQ_OFF)
#define LS2K_TOY_INT3_IRQ		(43 + LS2K_IRQ_OFF)

#define LS2K_DMA0_IRQ			(44 + LS2K_IRQ_OFF)
#define LS2K_DMA1_IRQ			(45 + LS2K_IRQ_OFF)
#define LS2K_DMA2_IRQ			(46 + LS2K_IRQ_OFF)
#define LS2K_DMA3_IRQ			(47 + LS2K_IRQ_OFF)
#define LS2K_DMA4_IRQ			(48 + LS2K_IRQ_OFF)

#define LS2K_OTG_IRQ			(49 + LS2K_IRQ_OFF)
#define LS2K_EHCI_IRQ			(50 + LS2K_IRQ_OFF)
#define LS2K_OHCI_IRQ			(51 + LS2K_IRQ_OFF)

#define LS2K_RTC_INT0_IRQ		(52 + LS2K_IRQ_OFF)
#define LS2K_RTC_INT1_IRQ		(53 + LS2K_IRQ_OFF)
#define LS2K_RTC_INT2_IRQ		(54 + LS2K_IRQ_OFF)

#define LS2K_RSA_IRQ			(55 + LS2K_IRQ_OFF)
#define LS2K_AES_IRQ			(56 + LS2K_IRQ_OFF)
#define LS2K_DES_IRQ			(57 + LS2K_IRQ_OFF)

#define LS2K_GPIO_LO_IRQ		(58 + LS2K_IRQ_OFF)
#define LS2K_GPIO_HI_IRQ		(59 + LS2K_IRQ_OFF)

#define LS2K_GPIO_0_IRQ			(60 + LS2K_IRQ_OFF)
#define LS2K_GPIO_1_IRQ			(61 + LS2K_IRQ_OFF)
#define LS2K_GPIO_2_IRQ			(62 + LS2K_IRQ_OFF)
#define LS2K_GPIO_3_IRQ			(63 + LS2K_IRQ_OFF)


#define LS2K_LAST_IRQ			(64 + LS2K_IRQ_OFF)


#endif /* _LS2K_INT_H */
