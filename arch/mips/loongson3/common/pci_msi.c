#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/msi.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <south-bridge.h>

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
		ret = ls_south_bridge->sb_setup_msi_irq(dev, entry);
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
        return ls_south_bridge->sb_setup_msi_irq(pdev, desc);
}

void arch_teardown_msi_irq(unsigned int irq)
{
	ls_south_bridge->sb_teardown_msi_irq(irq);
}
