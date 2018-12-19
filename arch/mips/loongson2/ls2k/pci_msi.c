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
#include <ls2k.h>
#include <ls2k_int.h>


void enable_ls2k_board_irq(struct irq_data *d);
void disable_ls2k_board_irq(struct irq_data *d);
void ack_ls2k_board_irq(struct irq_data *d);
int ls2k_set_irq_affinity(struct irq_data *d, const struct cpumask *affinity, bool force);

#define IRQ_LS2H_MSI_0 (LS2K_IRQ_OFF)
#define LS2H_NUM_MSI_IRQS 64
/*msi use irq, other device not used*/
static DECLARE_BITMAP(msi_irq_in_use, LS2H_NUM_MSI_IRQS)={LS2K_IRQ_MASK};


static DEFINE_SPINLOCK(lock);

#define irq2bit(irq) (irq - IRQ_LS2H_MSI_0)
#define bit2irq(bit) (IRQ_LS2H_MSI_0 + bit)

/*
 * Dynamic irq allocate and deallocation
 */
static int ls2k_create_irq(void)
{
	int irq, pos;
	unsigned long flags;

	spin_lock_irqsave(&lock, flags);
again:
	pos = find_first_zero_bit(msi_irq_in_use, LS2H_NUM_MSI_IRQS);
	if(pos==LS2H_NUM_MSI_IRQS)
	{
		spin_unlock_irqrestore(&lock, flags);
		return -ENOSPC;
	}

	irq = bit2irq(pos);
	/* test_and_set_bit operates on 32-bits at a time */
	if (test_and_set_bit(pos, msi_irq_in_use))
		goto again;
	spin_unlock_irqrestore(&lock, flags);

	dynamic_irq_init(irq);

	return irq;
}

static void ls2k_destroy_irq(unsigned int irq)
{
	int pos = irq2bit(irq);

	dynamic_irq_cleanup(irq);

	clear_bit(pos, msi_irq_in_use);
}

void ls2k_teardown_msi_irq(unsigned int irq)
{
	ls2k_destroy_irq(irq);
}

void disable_ls2k_msi_irq(struct irq_data *d)
{
	unsigned int irq = d->irq;
	int pos = irq2bit(irq);
	int reg = pos<32?LS2K_INT_MSI_TRIGGER_EN_0:LS2K_INT_MSI_TRIGGER_EN_1;
	disable_ls2k_board_irq(d);
	mask_msi_irq(d);
	ls2k_writel(ls2k_readl(reg)&~(1<<(pos&0x1f)), reg);
}

void enable_ls2k_msi_irq(struct irq_data *d)
{
	unsigned int irq = d->irq;
	int pos = irq2bit(irq);
	int reg = pos<32?LS2K_INT_MSI_TRIGGER_EN_0:LS2K_INT_MSI_TRIGGER_EN_1;
	enable_ls2k_board_irq(d);
	unmask_msi_irq(d);
	ls2k_writel(ls2k_readl(reg)|(1<<(pos&0x1f)), reg);
}


static struct irq_chip ls2k_msi_chip = {
	.name = "PCI-MSI",
	.irq_ack	= ack_ls2k_board_irq,
	.irq_mask	= disable_ls2k_msi_irq,
	.irq_unmask	= enable_ls2k_msi_irq,
	.irq_eoi	= enable_ls2k_msi_irq,
	.irq_set_affinity	= ls2k_set_irq_affinity,
};


int ls2k_setup_msi_irq(struct pci_dev *pdev, struct msi_desc *desc)
{
	int pos, irq = ls2k_create_irq();
	struct msi_msg msg;


	if (irq < 0)
		return irq;

	pos = irq2bit(irq);
	irq_set_msi_desc(irq, desc);
	msg.address_hi = 0;
	msg.address_lo = pos<32?LS2K_INT_MSI_TRIGGER_0:LS2K_INT_MSI_TRIGGER_1;

	/*irq dispatch to ht vector 1,2.., 0 for leagacy devices*/
	msg.data = pos&0x1f;


	printk("irq=%d\n", irq);
	write_msi_msg(irq, &msg);
	irq_set_chip_and_handler(irq, &ls2k_msi_chip, handle_level_irq);

	return 0;
}


static bool nomsix=1;
core_param(nomsix, nomsix, bool, 0664);
static bool nomsi=0;
core_param(nomsi, nomsi, bool, 0664);

int arch_setup_msi_irqs(struct pci_dev *dev, int nvec, int type)
{
	struct msi_desc *entry;
	int ret;

	if ((type == PCI_CAP_ID_MSIX && nomsix) || (type == PCI_CAP_ID_MSI && nomsi))
		return -ENOSPC;
	/*
	 * If an architecture wants to support multiple MSI, it needs to
	 * override arch_setup_msi_irqs()
	 */
	if (type == PCI_CAP_ID_MSI && nvec > 1)
		return 1;

	list_for_each_entry(entry, &dev->msi_list, list) {
		ret = ls2k_setup_msi_irq(dev, entry);
		if (ret < 0)
			return ret;
		if (ret > 0)
			return -ENOSPC;
	}

	return 0;
}

int arch_setup_msi_irq(struct pci_dev *pdev, struct msi_desc *desc)
{
	if(nomsi)
	 return -ENOSPC;
        return ls2k_setup_msi_irq(pdev, desc);
}

void arch_teardown_msi_irq(unsigned int irq)
{
	ls2k_teardown_msi_irq(irq);
}
