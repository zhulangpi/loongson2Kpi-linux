#include <linux/mm.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/swiotlb.h>
#include <linux/bootmem.h>

#include <asm/bootinfo.h>
#include <asm/dma-mapping.h>
#include <dma-coherence.h>

static inline void *dma_addr_to_virt(struct device *dev,
	dma_addr_t dma_addr)
{
	unsigned long addr = plat_dma_addr_to_phys(dev, dma_addr);

	return phys_to_virt(addr);
}

static void *loongson_dma_alloc_coherent(struct device *dev, size_t size,
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
	else if (dev->coherent_dma_mask <= DMA_BIT_MASK(32))
		gfp |= __GFP_DMA32;
	else
#endif
	;
	gfp |= __GFP_NORETRY | __GFP_NOWARN;

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

static void loongson_dma_free_coherent(struct device *dev, size_t size,
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

static dma_addr_t loongson_dma_map_page(struct device *dev, struct page *page,
				unsigned long offset, size_t size,
				enum dma_data_direction dir,
				struct dma_attrs *attrs)
{
	dma_addr_t daddr = swiotlb_map_page(dev, page, offset, size,
					dir, attrs);
	if (!plat_device_is_coherent(dev)) {
		dma_cache_sync(NULL, dma_addr_to_virt(dev,
				daddr), size, dir);
	}
	mb();
	return daddr;
}

static void loongson_dma_unmap_page(struct device *hwdev, dma_addr_t dev_addr,
			size_t size, enum dma_data_direction dir,
			struct dma_attrs *attrs)
{
	if (!plat_device_is_coherent(hwdev))
		dma_cache_sync(NULL, dma_addr_to_virt(hwdev, dev_addr), size,
				dir);
	swiotlb_unmap_page(hwdev, dev_addr, size, dir, attrs);
}

static int loongson_dma_map_sg(struct device *dev, struct scatterlist *sgl,
				int nents, enum dma_data_direction dir,
				struct dma_attrs *attrs)
{
	struct scatterlist *sg;
	int i;
	int r = swiotlb_map_sg_attrs(dev, sgl, nents, dir, NULL);

	if (!plat_device_is_coherent(dev))
	{
		for_each_sg(sgl, sg, nents, i) {
			dma_cache_sync(NULL, dma_addr_to_virt(dev,
					sg->dma_address), sg->length, dir);
		}
	}
	mb();

	return r;
}

static void loongson_dma_unmap_sg_attrs(struct device *hwdev, struct scatterlist *sgl,
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

static void loongson_dma_sync_single_for_cpu(struct device *hwdev, dma_addr_t dev_addr,
			    size_t size, enum dma_data_direction dir)
{
	if (!plat_device_is_coherent(hwdev))
		dma_cache_sync(NULL, dma_addr_to_virt(hwdev, dev_addr), size, dir);
	swiotlb_sync_single_for_cpu(hwdev, dev_addr, size, dir);
}

static void loongson_dma_sync_single_for_device(struct device *dev,
				dma_addr_t dma_handle, size_t size,
				enum dma_data_direction dir)
{
	swiotlb_sync_single_for_device(dev, dma_handle, size, dir);
	if (!plat_device_is_coherent(dev))
		dma_cache_sync(NULL, dma_addr_to_virt(dev, dma_handle), size, dir);
	mb();
}

static void loongson_dma_sync_sg_for_cpu(struct device *hwdev, struct scatterlist *sgl,
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

static void loongson_dma_sync_sg_for_device(struct device *dev,
				struct scatterlist *sgl, int nents,
				enum dma_data_direction dir)
{
	struct scatterlist *sg;
	int i;

	swiotlb_sync_sg_for_device(dev, sgl, nents, dir);
	if (!plat_device_is_coherent(dev))
	{
		for_each_sg(sgl, sg, nents, i) {
			dma_cache_sync(NULL, dma_addr_to_virt(dev,
					sg->dma_address), sg->length, dir);
		}
	}
	mb();
}

static int loongson_dma_set_mask(struct device *dev, u64 mask)
{
	if (!dma64_supported) {
		if (mask > DMA_BIT_MASK(32)) {
			*dev->dma_mask = DMA_BIT_MASK(32);
			return -EIO;
		}
	}

	*dev->dma_mask = mask;

	return 0;
}

static dma_addr_t loongson_unity_phys_to_dma32(struct device *dev, phys_addr_t paddr)
{
	paddr = paddr < 0x10000000 ?
		(paddr | 0x0000000080000000) : paddr;

	return paddr;
}

static phys_addr_t loongson_unity_dma_to_phys32(struct device *dev, dma_addr_t daddr)
{
	daddr = (daddr < 0x90000000 && daddr >= 0x80000000) ?
		daddr & 0x0fffffff : daddr;

	return daddr;
}

static struct mips_dma_map_ops loongson_linear_dma_map_ops = {
	.dma_map_ops = {
		.alloc = loongson_dma_alloc_coherent,
		.free = loongson_dma_free_coherent,
		.map_page = loongson_dma_map_page,
		.unmap_page = loongson_dma_unmap_page,
		.map_sg = loongson_dma_map_sg,
		.unmap_sg = loongson_dma_unmap_sg_attrs,
		.sync_single_for_cpu = loongson_dma_sync_single_for_cpu,
		.sync_single_for_device = loongson_dma_sync_single_for_device,
		.sync_sg_for_cpu = loongson_dma_sync_sg_for_cpu,
		.sync_sg_for_device = loongson_dma_sync_sg_for_device,
		.mapping_error = swiotlb_dma_mapping_error,
		.dma_supported = swiotlb_dma_supported,
		.set_dma_mask = loongson_dma_set_mask
	},
	.phys_to_dma = mips_unity_phys_to_dma,
	.dma_to_phys = mips_unity_dma_to_phys
};

void __init rs780_plat_swiotlb_setup(void)
{
	if(!dma64_supported) {
		loongson_linear_dma_map_ops.phys_to_dma = loongson_unity_phys_to_dma32;
		loongson_linear_dma_map_ops.dma_to_phys = loongson_unity_dma_to_phys32;
		pr_info("swiotlb:restricted 32bit dma!\n");
	} 
	swiotlb_init(1);
	loongson_dma_map_ops = &loongson_linear_dma_map_ops;
}
