/*
reference: arch/arm/mach-iop13xx/msi.c
generic_handle_irq(irq)

ht 40bit地址 0xfd_f800_00xx
xx为向量号
*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/msi.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/pci.h>

#include <pci.h>
/*
 * Shifts for MSI data
 */

#define MSI_DATA_VECTOR_SHIFT		0
#define  MSI_DATA_VECTOR_MASK		0x000000ff
#define	 MSI_DATA_VECTOR(v)		(((v) << MSI_DATA_VECTOR_SHIFT) & \
					 MSI_DATA_VECTOR_MASK)

#define MSI_DATA_DELIVERY_MODE_SHIFT	8
#define  MSI_DATA_DELIVERY_FIXED	(0 << MSI_DATA_DELIVERY_MODE_SHIFT)
#define  MSI_DATA_DELIVERY_LOWPRI	(1 << MSI_DATA_DELIVERY_MODE_SHIFT)

#define MSI_DATA_LEVEL_SHIFT		14
#define	 MSI_DATA_LEVEL_DEASSERT	(0 << MSI_DATA_LEVEL_SHIFT)
#define	 MSI_DATA_LEVEL_ASSERT		(1 << MSI_DATA_LEVEL_SHIFT)

#define MSI_DATA_TRIGGER_SHIFT		15
#define  MSI_DATA_TRIGGER_EDGE		(0 << MSI_DATA_TRIGGER_SHIFT)
#define  MSI_DATA_TRIGGER_LEVEL		(1 << MSI_DATA_TRIGGER_SHIFT)

/*
 * Shift/mask fields for msi address
 */

#define MSI_ADDR_BASE_HI		0
#define MSI_ADDR_BASE_LO		0xfee00000

#define MSI_ADDR_DEST_MODE_SHIFT	2
#define  MSI_ADDR_DEST_MODE_PHYSICAL	(0 << MSI_ADDR_DEST_MODE_SHIFT)
#define	 MSI_ADDR_DEST_MODE_LOGICAL	(1 << MSI_ADDR_DEST_MODE_SHIFT)

#define MSI_ADDR_REDIRECTION_SHIFT	3
#define  MSI_ADDR_REDIRECTION_CPU	(0 << MSI_ADDR_REDIRECTION_SHIFT)
					/* dedicated cpu */
#define  MSI_ADDR_REDIRECTION_LOWPRI	(1 << MSI_ADDR_REDIRECTION_SHIFT)
					/* lowest priority */

#define MSI_ADDR_DEST_ID_SHIFT		12
#define	 MSI_ADDR_DEST_ID_MASK		0x00ffff0
#define  MSI_ADDR_DEST_ID(dest)		(((dest) << MSI_ADDR_DEST_ID_SHIFT) & \
					 MSI_ADDR_DEST_ID_MASK)
#define MSI_ADDR_EXT_DEST_ID(dest)	((dest) & 0xffffff00)

#define MSI_ADDR_IR_EXT_INT		(1 << 4)
#define MSI_ADDR_IR_SHV			(1 << 3)
#define MSI_ADDR_IR_INDEX1(index)	((index & 0x8000) >> 13)
#define MSI_ADDR_IR_INDEX2(index)	((index & 0x7fff) << 5)

//#define LS3A_HT_PCI_MSIX_MSGADDR 	0xfee00000 //0xfee00000
//#define LS3A_HT_PCI_MSIX_MSGDATA0 	0x00000000 //0x00080000

#define LS3A_HT_IRQ_0 			64
#define LS3A_HT_IRQ_MAX 		255
#define LS3A_HT_MSI_VECTOR_0 	64 	//32 //low 32 bit reserved for lagecy int
#define LS3A_HT_MSI_VECTOR_MAX 	255


#define IRQ_LS3A_MSI_0 64
#define LS3A_NUM_MSI_IRQS 256
//static DECLARE_BITMAP(msi_irq_in_use, LS3A_NUM_MSI_IRQS) = {0xffffffffFfffffffULL,0,};
//static DECLARE_BITMAP(msi_irq_in_use, LS3A_NUM_MSI_IRQS) = {0xffffffffFfffffffULL,0xffffffffFfffffffULL,0xffffffffFfffffffULL,0};
static DECLARE_BITMAP(msi_irq_in_use, LS3A_NUM_MSI_IRQS) = {0xffffffffULL,0,};


static DEFINE_SPINLOCK(lock);
extern int ls3a_ht_stripe4;
extern int ht_irq_mask[8];

#define irq2ht(irq) (irq - IRQ_LS3A_MSI_0)
#define irq2bit(irq) (irq - IRQ_LS3A_MSI_0)
#define bit2irq(bit) (IRQ_LS3A_MSI_0 + bit)
/*
 * Dynamic irq allocate and deallocation
 */
static int balance;
static int rs780e_create_irq(void)
{
	int irq, pos;
	unsigned long flags;

	spin_lock_irqsave(&lock, flags);
again:
	pos = find_first_zero_bit(&msi_irq_in_use[balance], 64);
	if(pos==64)
	{
		balance = (balance+1)&3;
		goto again;
	}
	pos = pos + balance*64;
	irq = bit2irq(pos);
	if (irq > NR_IRQS)
	{	
	spin_unlock_irqrestore(&lock, flags);
		return -ENOSPC;
	}
	/* test_and_set_bit operates on 32-bits at a time */
	if (test_and_set_bit(pos, msi_irq_in_use))
		goto again;
	 //balance = (balance+1)&3;
	spin_unlock_irqrestore(&lock, flags);

	dynamic_irq_init(irq);

	return irq;
}

static void rs780e_destroy_irq(unsigned int irq)
{
	int pos = irq2ht(irq);
	int bit = irq2bit(irq);

	*(volatile unsigned int *)(ht_control_base + 0xA0 + (pos/32)*4) &= ~(1<<(pos%32));
	ht_irq_mask[pos/32] &= ~(1<<(pos%32));
	dynamic_irq_cleanup(irq);

	clear_bit(bit, msi_irq_in_use);
}

void rs780e_teardown_msi_irq(unsigned int irq)
{
	rs780e_destroy_irq(irq);
}

static void ls3a_msi_nop(struct irq_data *data)
{
	return;
}

static struct irq_chip ls3a_msi_chip = {
	.name = "PCI-MSI",
	.irq_ack = ls3a_msi_nop,
	.irq_enable = unmask_msi_irq,
	.irq_disable = mask_msi_irq,
	.irq_mask = mask_msi_irq,
	.irq_unmask = unmask_msi_irq,
};

int rs780e_setup_msi_irq(struct pci_dev *pdev, struct msi_desc *desc)
{
	int pos, irq = rs780e_create_irq();
	struct msi_msg msg;

	if (irq < 0)
		return irq;

	pos = irq2ht(irq);
	irq_set_msi_desc(irq, desc);
#if 1
	msg.address_hi = MSI_ADDR_BASE_HI;
	msg.address_lo = MSI_ADDR_BASE_LO;

	/*irq dispatch to ht vector 1,2.., 0 for leagacy devices*/
	msg.data = MSI_DATA_VECTOR(pos);

#else
	msg.address_hi = 0;
	msg.address_lo = 0xfee00000;

/*irq dispatch to ht vector 1,2.., 0 for leagacy devices*/
	//msg.data = 0x8000|(pos+32);
	msg.data = 0x8000|pos;
#endif
	*(volatile unsigned int *)(ht_control_base + 0xA0 + (pos/32)*4) |= 1<<(pos%32);
	ht_irq_mask[pos/32] |= 1<<(pos%32);

	printk("irq=%d\n", irq);
	write_msi_msg(irq, &msg);
	irq_set_chip_and_handler(irq, &ls3a_msi_chip, handle_simple_irq);

	return 0;
}

