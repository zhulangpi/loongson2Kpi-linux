/*
 * fixup-ls2ksoc.c
 *
 * Copyright (C) 2004 ICT CAS
 * Author: Li xiaoyu, ICT CAS
 *   lixy@ict.ac.cn
 *
 * Copyright (C) 2007 Lemote, Inc. & Institute of Computing Technology
 * Author: Fuxin Zhang, zhangfx@lemote.com
 *
 * Copyright (C) 2010 Dawning
 * Author: Yongcheng Li, Dawning
 *    liych@dawning.com.cn
 *
 * Copyright (C) 2010 Dawning
 * Author: Lv minqiang, Dawning, Inc
 *    lvmq@dawning.com.cn
 *  Changed for :
 *		1.addust coding
 *		2. add sata fixup
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#include <linux/init.h>
#include <linux/pci.h>
#include <ls2k.h>
#include <ls2k_int.h>

int __init pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{

	struct pci_bus  *bus = dev->bus;
	unsigned char busnum = dev->bus->number;
        int fn = dev->devfn & 7;
	int irq = 0;

	if(busnum == 0)
	{
	  switch(slot)
	  {
		default:
		case 2:
		/*APB 2*/
		 irq = 0;
		break;

		case 3:
		/*GMAC0 3 0*/
		/*GMAC1 3 1*/
		irq = (fn==0)?12:14;
		break;

		case 4:
		/*
		OTG: 4 0
		EHCI: 4 1
		OHCI: 4 2
		*/
		 irq = (fn == 0)? 49:(fn == 1)? 50:51;
		break;

		case 5:
		/*GPU*/
		 irq = 29;
		break;

		case 6:
		/*DC*/
		 irq = 28;
		break;

		case 7:
		/*HDA*/
		 irq = 4;
		break;

		case 8:
		/*SATA*/
		 irq = 19;
		break;

		case 9:
		/*PCIE PORT 0*/
		 irq = 32+pin-1;;
		break;

		case 10:
		/*PCIE PORT 1*/
		 irq = 33+pin-1;
		break;

		case 11:
		/*PCIE PORT 2*/
		 irq = 34+pin-1;
		break;

		case 12:
		/*PCIE PORT 3*/
		 irq = 35+pin-1;
		break;

		case 13:
		/*PCIE1 PORT 0*/
		 irq = 36+pin-1;
		break;

		case 14:
		/*PCIE1 PORT 1*/
		 irq = 37+pin-1;
		break;

		case 15:
		/*DMA*/
		break;

	  }
	}
	else
	{

		while(bus->parent->parent)
		  bus = bus->parent;

                slot = bus->self->devfn >> 3;
		/* for non bridge single function dev  */
		pin = slot - 9;
		irq = 32 + pin;
		
	}

	return LS2K_IRQ_BASE+irq;
}

extern struct mips_dma_map_ops mips_default_dma_map_ops;

/* Do platform specific device initialization at pci_enable_device() time */
int pcibios_plat_dev_init(struct pci_dev *dev)
{

	int pos;
	u16 max_payload_spt, cur_payload_spt, control;

	/**
	 * fixup settings of MPS & MRRS during fixing irq
	 * check whether MPSSPT is smaller than parents',
	 * keep the smaller MPSSPT in the child's register
	 */
	if (!(dev->bus->parent)) {
		/*gmac can support 64bit dma*/
		if(PCI_SLOT(dev->devfn) == 3) dev->dev.archdata.dma_ops = &mips_default_dma_map_ops;

		pos = pci_find_capability(dev, PCI_CAP_ID_EXP);
		if (!pos) return 0;
		pci_read_config_word(dev, pos + PCI_EXP_DEVCAP,
				     &max_payload_spt);
		max_payload_spt &= PCI_EXP_DEVCAP_PAYLOAD;
	} else {
		pos = pci_find_capability(dev->bus->self, PCI_CAP_ID_EXP);
		if (!pos) return 0;
		pci_read_config_word(dev->bus->self, pos + PCI_EXP_DEVCAP,
				     &max_payload_spt);
		max_payload_spt &= PCI_EXP_DEVCAP_PAYLOAD;

		pos = pci_find_capability(dev, PCI_CAP_ID_EXP);
		pci_read_config_word(dev, pos + PCI_EXP_DEVCAP,
				     &cur_payload_spt);
		cur_payload_spt &= PCI_EXP_DEVCAP_PAYLOAD;

		if (max_payload_spt > cur_payload_spt)
			max_payload_spt = cur_payload_spt;
	}

	max_payload_spt = 1;

	pci_read_config_word(dev, pos + PCI_EXP_DEVCTL, &control);
	control &= (~PCI_EXP_DEVCTL_PAYLOAD & ~PCI_EXP_DEVCTL_READRQ);
	control |= ((max_payload_spt << 5) | (max_payload_spt << 12));
	pci_write_config_word(dev, pos + PCI_EXP_DEVCTL, control);
	pr_info("pci %s: set Max_Payload_Size & Max_Read_Request_Size to %03x\n",
	       pci_name(dev), max_payload_spt);

	dev->dev.archdata.dma_ops = &mips_default_dma_map_ops;

	return 0;
}
