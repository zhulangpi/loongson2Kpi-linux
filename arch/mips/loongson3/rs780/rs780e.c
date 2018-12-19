
#include <south-bridge.h>
#include <loongson.h>

extern void rs780_init_irq(void);
extern void rs780_irq_dispatch(unsigned int pending);
extern int loongson_rtc_platform_init(void);
extern u64 io_base_regs_addr;
extern int rs780e_setup_msi_irq(struct pci_dev *pdev, struct msi_desc *desc);
extern void rs780e_teardown_msi_irq(unsigned int irq);

#ifdef CONFIG_SWIOTLB
extern void rs780_plat_swiotlb_setup(void);
#endif

extern void rs780_pcibios_init(void);

static void __init rs780_arch_initcall(void)
{
	rs780_pcibios_init();
}

static void __init rs780_device_initcall(void)
{
	loongson_rtc_platform_init();
}

void __init rs780_early_config(void)
{
	if (cputype == Loongson_3A) {
		switch (nr_cpus_loongson) {
			case 8:
				/* Loongson-3A1000 dual-way do not support
				 * inter-node uncache DMA */
				if ((read_c0_prid() & 0xf) == PRID_REV_LOONGSON3A)
					return ;
				/*Node 1*/
				HT_uncache_enable_reg2	= HT_rx_win_enable_reg2;
				HT_uncache_base_reg2	= HT_rx_win_base_reg2;
				break;
			case 16:
				/* Loongson-3A 4way board non-coherent DMA configuration */
				/* Not used in PMON, to make Node 3 addr */
				North_win0_base = 0x0000202000000000;
				North_win0_mask = 0xffffffe000000000;
				North_win0_mmap = 0x0000300000000086;
				/*Node 1*/
				HT_uncache_enable_reg2	= HT_rx_win_enable_reg2;
				HT_uncache_base_reg2	= HT_rx_win_base_reg2;
				/*Node 2 and Node 3*/
				HT_uncache_enable_reg3	= 0xC0200000;
				HT_uncache_base_reg3	= 0x4000C000;
				break;
			default:
				break;
		}

		HT_uncache_enable_reg0	= HT_rx_win_enable_reg0;
		HT_uncache_base_reg0	= HT_rx_win_base_reg0;

		HT_uncache_enable_reg1	= HT_rx_win_enable_reg1;
		HT_uncache_base_reg1	= HT_rx_win_base_reg1;
	} else if(cputype == Loongson_3B){
		switch (nr_cpus_loongson) {
			case 6 ... 8:
				/* set HT-access uncache */
				HT_uncache_enable_reg0	= 0xc0000000;
				HT_uncache_base_reg0	= 0x0080fff0;

				HT_uncache_enable_reg1	= 0xc0000000;
				HT_uncache_base_reg1	= 0x00008000;
				writeq(0x0000002000000000, (void *)0x900010003ff06700);
				writeq(0xffffffe000000000, (void *)0x900010003ff06740);
				writeq(0x00001000000000f0, (void *)0x900010003ff06780);
				break;
			default:
				/* Loongson-3B dual-way do not support inter-node uncache DMA */
				break;
		}
	}
	__sync();
}

const struct south_bridge rs780_south_bridge = {
#ifdef CONFIG_DMA_NONCOHERENT
	.sb_early_config	= rs780_early_config,
#endif
	.sb_init_irq		= rs780_init_irq,
#ifdef CONFIG_SWIOTLB
	.sb_init_swiotlb	= rs780_plat_swiotlb_setup,
#endif
	.sb_irq_dispatch	= rs780_irq_dispatch,
	.sb_arch_initcall	= rs780_arch_initcall,
	.sb_device_initcall	= rs780_device_initcall,
#ifdef CONFIG_PCI_MSI
	.sb_setup_msi_irq	= rs780e_setup_msi_irq,
	.sb_teardown_msi_irq	= rs780e_teardown_msi_irq,
#endif
};
