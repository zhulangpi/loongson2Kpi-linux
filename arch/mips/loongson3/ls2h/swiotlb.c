#include <linux/types.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/swiotlb.h>
#include <linux/scatterlist.h>

#include <asm/bootinfo.h>

#include <dma-coherence.h>

#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/string.h>
#include <linux/gfp.h>

#include <asm/cache.h>
#include <asm/dma-mapping.h>
#include <asm/io.h>

extern void __dma_sync(unsigned long addr, size_t size,
	enum dma_data_direction direction);

static inline void *dma_addr_to_virt(struct device *dev,
	dma_addr_t dma_addr)
{
	unsigned long addr = plat_dma_addr_to_phys(dev, dma_addr);

	return phys_to_virt(addr);
}

static dma_addr_t pcie_dma_map_page(struct device *dev, struct page *page,
	unsigned long offset, size_t size, enum dma_data_direction direction,
	struct dma_attrs *attrs)
{
	dma_addr_t daddr;

	BUG_ON(direction == DMA_NONE);

	daddr = swiotlb_map_page(dev, page, offset, size,
						direction, attrs);

	if (!plat_device_is_coherent(dev)) {
		dma_cache_sync(NULL, phys_to_virt(plat_dma_addr_to_phys(dev,
				daddr)), size, direction);
	}

	mb();
	return daddr;
}

void pcie_dma_unmap_page(struct device *hwdev, dma_addr_t dev_addr,
			size_t size, enum dma_data_direction dir,
			struct dma_attrs *attrs)
{
	if (!plat_device_is_coherent(hwdev))
		dma_cache_sync(NULL, dma_addr_to_virt(hwdev, dev_addr), size,
				dir);
	swiotlb_unmap_page(hwdev, dev_addr, size, dir, attrs);
}

static int pcie_dma_map_sg(struct device *dev, struct scatterlist *sgl,
	int nents, enum dma_data_direction direction, struct dma_attrs *attrs)
{
	struct scatterlist *sg;
	int i;
	int r = swiotlb_map_sg_attrs(dev, sgl, nents, direction, NULL);
	if (!plat_device_is_coherent(dev))
	{
		for_each_sg(sgl, sg, nents, i) {
			dma_cache_sync(NULL, dma_addr_to_virt(dev,
					sg->dma_address), sg->length, direction);
		}
	}
	mb();
	return r;
}

static void
pcie_dma_unmap_sg_attrs(struct device *hwdev, struct scatterlist *sgl,
			int nelems, enum dma_data_direction dir,
			struct dma_attrs *attrs)
{
	struct scatterlist *sg;
	int i;
	if (!plat_device_is_coherent(hwdev) &&
			dir != DMA_TO_DEVICE) {
		for_each_sg(sgl, sg, nelems, i) {
				dma_cache_sync(NULL, dma_addr_to_virt(hwdev,
						sg->dma_address),sg->length, dir);
		}
	}
	swiotlb_unmap_sg_attrs(hwdev, sgl, nelems, dir, attrs);
}

static void
pcie_dma_sync_single_for_cpu(struct device *hwdev, dma_addr_t dev_addr,
				size_t size, enum dma_data_direction dir)
{
	if (!plat_device_is_coherent(hwdev))
		dma_cache_sync(NULL, dma_addr_to_virt(hwdev, dev_addr), size, dir);
	swiotlb_sync_single_for_cpu(hwdev, dev_addr, size, dir);
}

static void pcie_dma_sync_single_for_device(struct device *dev,
	dma_addr_t dma_handle, size_t size, enum dma_data_direction direction)
{
	swiotlb_sync_single_for_device(dev, dma_handle, size, direction);
	if (!plat_device_is_coherent(dev))
		dma_cache_sync(NULL, dma_addr_to_virt(dev, dma_handle), size, direction);
	mb();
}

static void
pcie_dma_sync_sg_for_cpu(struct device *hwdev, struct scatterlist *sgl,
			int nelems, enum dma_data_direction dir)
{
	struct scatterlist *sg;
	int i;

	if (!plat_device_is_coherent(hwdev))
	{
		for_each_sg(sgl, sg, nelems, i) {
			dma_cache_sync(NULL, dma_addr_to_virt(hwdev,
					sg->dma_address), sg->length, dir);
		}
	}

	swiotlb_sync_sg_for_cpu(hwdev, sgl, nelems, dir);
}

static void pcie_dma_sync_sg_for_device(struct device *dev,
	struct scatterlist *sgl, int nelems, enum dma_data_direction direction)
{
	struct scatterlist *sg;
	int i;

	swiotlb_sync_sg_for_device(dev, sgl, nelems, direction);

	if (!plat_device_is_coherent(dev))
	{
		for_each_sg(sgl, sg, nelems, i) {
			dma_cache_sync(NULL, dma_addr_to_virt(dev,
					sg->dma_address), sg->length, direction);
		}
	}
	mb();
}

static void *pcie_dma_alloc_coherent(struct device *dev, size_t size,
	dma_addr_t *dma_handle, gfp_t gfp, struct dma_attrs *attrs)
{
	void *ret;

	if (dma_alloc_from_coherent(dev, size, dma_handle, &ret))
		return ret;

	/* ignore region specifiers */
	gfp &= ~(__GFP_DMA | __GFP_DMA32 | __GFP_HIGHMEM);

#ifdef CONFIG_ZONE_DMA
	if (dev == NULL)
		gfp |= __GFP_DMA;
	else if (dev->coherent_dma_mask <= DMA_BIT_MASK(24))
		gfp |= __GFP_DMA;
	else
#endif
#ifdef CONFIG_ZONE_DMA32
	if (dev == NULL)
		gfp |= __GFP_DMA32;
	/* Loongson3 doesn't support DMA above 40-bit */
	else if (dev->coherent_dma_mask <= DMA_BIT_MASK(40))
		gfp |= __GFP_DMA32;
	else
#endif
		;

	/* Don't invoke OOM killer */
	gfp |= __GFP_NORETRY;

	ret = swiotlb_alloc_coherent(dev, size, dma_handle, gfp);
	if(ret)
	{
		if (!plat_device_is_coherent(dev)) {
			dma_cache_sync(NULL, dma_addr_to_virt(dev,
					*dma_handle), size, DMA_TO_DEVICE);
			ret = UNCAC_ADDR(ret);
		}
	}

	mb();
	return ret;
}

static void pcie_dma_free_coherent(struct device *dev, size_t size,
	void *vaddr, dma_addr_t dma_handle, struct dma_attrs *attrs)
{
	int order = get_order(size);

	if (dma_release_from_coherent(dev, order, vaddr))
		return;

	if (!plat_device_is_coherent(dev)) {
		vaddr = CAC_ADDR(vaddr);
		dma_cache_sync(NULL, dma_addr_to_virt(dev,
				dma_handle), size, DMA_FROM_DEVICE);
	}

	swiotlb_free_coherent(dev, size, vaddr, dma_handle);
}

static int pcie_dma_set_mask(struct device *dev, u64 mask)
{
	/* Loongson3 doesn't support DMA above 40-bit */
	if (mask > DMA_BIT_MASK(32)) {
		*dev->dma_mask = DMA_BIT_MASK(32);
		return -EIO;
	}

	*dev->dma_mask = mask;

	return 0;
}
static dma_addr_t pcie_phys_to_dma(struct device *dev, phys_addr_t paddr)
{
	if (strstr(einter->description, "V3.3.1"))
		return (paddr < 0x10000000) ? paddr : (paddr - 0x80000000);
	else
		return paddr & 0xffffffff;
}

static phys_addr_t pcie_dma_to_phys(struct device *dev, dma_addr_t daddr)
{
	if (strstr(einter->description, "V3.3.1"))
		return (daddr < 0x10000000) ? daddr : (daddr + 0x80000000);
	else
		return (daddr < 0x10000000) ? daddr : (daddr | 0x100000000);
}

struct mips_dma_map_ops ls2h_pcie_dma_map_ops = {
	.dma_map_ops = {
		.alloc		= pcie_dma_alloc_coherent,
		.free		= pcie_dma_free_coherent,
		.map_page		= pcie_dma_map_page,
		.unmap_page		= pcie_dma_unmap_page,
		.map_sg			= pcie_dma_map_sg,
		.unmap_sg		= pcie_dma_unmap_sg_attrs,
		.sync_single_for_cpu	= pcie_dma_sync_single_for_cpu,
		.sync_single_for_device	= pcie_dma_sync_single_for_device,
		.sync_sg_for_cpu	= pcie_dma_sync_sg_for_cpu,
		.sync_sg_for_device	= pcie_dma_sync_sg_for_device,
		.mapping_error		= swiotlb_dma_mapping_error,
		.dma_supported		= swiotlb_dma_supported,
		.set_dma_mask 		= pcie_dma_set_mask
	},
	.phys_to_dma	= pcie_phys_to_dma,
	.dma_to_phys	= pcie_dma_to_phys
};

void __init ls2h_init_swiotlb(void)
{
	extern int swiotlb_force;
	swiotlb_force = 1;
	swiotlb_init(1);
}
