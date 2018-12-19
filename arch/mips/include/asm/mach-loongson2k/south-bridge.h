#include <linux/types.h>
#include <linux/pci.h>

struct south_bridge {
	void	(*sb_early_config)(void);
	void	(*sb_init_irq)(void);
	void	(*sb_init_swiotlb)(void);
	void	(*sb_irq_dispatch)(unsigned int pending);
	int	(*sb_pcibios_init)(struct pci_dev *dev);
	int	(*sb_pcibios_map_irq)(struct pci_dev *dev, u8 slot, u8 pin);
	void	(*sb_arch_initcall)(void);
	void	(*sb_device_initcall)(void);
};

extern struct south_bridge *ls_south_bridge;

extern void south_bridge_register(struct south_bridge *sb);

