#ifndef _ASM_DMA_MAPPING_H
#define _ASM_DMA_MAPPING_H

#include <asm/scatterlist.h>
#include <asm/dma-coherence.h>
#include <asm/cache.h>
#include <asm-generic/dma-coherent.h>

#if defined(CONFIG_CPU_LOONGSON3)
extern unsigned long long io_tlb_start_addr;
extern unsigned long long io_tlb_end_addr;
#endif

struct mips_dma_map_ops {
	struct dma_map_ops dma_map_ops;
	dma_addr_t (*phys_to_dma)(struct device *dev, phys_addr_t paddr);
	phys_addr_t (*dma_to_phys)(struct device *dev, dma_addr_t daddr);
};

extern struct mips_dma_map_ops *loongson_dma_map_ops;

static inline bool special_dev(struct device *dev)
{
	return (strcmp(dev->driver->name,"igb") == 0) || (strcmp(dev->driver->name,"e1000e") == 0);
}

static inline struct dma_map_ops *get_dma_ops(struct device *dev)
{
	struct mips_dma_map_ops *ops;

#if !defined(CONFIG_LOONGSON_GUEST_OS) && !defined(CONFIG_CPU_LOONGSON2K)
	if (board_type == LS2H) {
		if (dev && dev->archdata.dma_ops && !special_dev(dev))
			ops = dev->archdata.dma_ops;
		else
			ops = loongson_dma_map_ops;
	} else
#endif
	{
		if (dev && dev->archdata.dma_ops)
			ops = dev->archdata.dma_ops;
		else
			ops = loongson_dma_map_ops;
	}

	return &ops->dma_map_ops;
}

static inline struct dma_map_ops *get_special_dma_ops(struct device *dev, dma_addr_t addr)
{
	struct mips_dma_map_ops *ops;

#if !defined(CONFIG_LOONGSON_GUEST_OS) && !defined(CONFIG_CPU_LOONGSON2K)
	if (board_type == LS2H) {
		if ((special_dev(dev) && (io_tlb_start_addr <= addr) && (addr < io_tlb_end_addr))
				|| (dev && dev->archdata.dma_ops && !special_dev(dev)))
			ops = dev->archdata.dma_ops;
		else
			ops = loongson_dma_map_ops;
	} else
#endif
	{
		if (dev && dev->archdata.dma_ops)
			ops = dev->archdata.dma_ops;
		else
			ops = loongson_dma_map_ops;
	}

	return &ops->dma_map_ops;
}

static inline struct dma_map_ops *get_dev_dma_ops(struct device *dev,
		dma_addr_t addr, enum dma_data_direction dir)
{
	struct mips_dma_map_ops *ops;

#if !defined(CONFIG_LOONGSON_GUEST_OS) && !defined(CONFIG_CPU_LOONGSON2K)
	if (board_type == LS2H) {
		if ((dev && dev->archdata.dma_ops && !special_dev(dev))
				|| ((((io_tlb_start_addr <= addr) && (addr < io_tlb_end_addr)) || (addr & 0xf))
					&& special_dev(dev) && (dir & DMA_TO_DEVICE)))
			ops = dev->archdata.dma_ops;
		else
			ops = loongson_dma_map_ops;
	} else
#endif
	{
		if (dev && dev->archdata.dma_ops)
			ops = dev->archdata.dma_ops;
		else
			ops = loongson_dma_map_ops;
	}

	return &ops->dma_map_ops;
}

static inline dma_addr_t phys_to_dma(struct device *dev, phys_addr_t paddr)
{
	struct mips_dma_map_ops *ops = container_of(get_dma_ops(dev),
						    struct mips_dma_map_ops,
						    dma_map_ops);

	return ops->phys_to_dma(dev, paddr);
}

static inline phys_addr_t dma_to_phys(struct device *dev, dma_addr_t daddr)
{
	struct mips_dma_map_ops *ops = container_of(get_dma_ops(dev),
						    struct mips_dma_map_ops,
						    dma_map_ops);

	return ops->dma_to_phys(dev, daddr);
}

static inline bool dma_capable(struct device *dev, dma_addr_t addr, size_t size)
{
	if (!dev->dma_mask)
		return 0;

#if defined(CONFIG_CPU_LOONGSON3)
	if (!dma64_supported)
		return addr + size <= 0x00000000ffffffff;
#endif

	return addr + size <= *dev->dma_mask;
}

static inline void dma_mark_clean(void *addr, size_t size) {}

#include <asm-generic/dma-mapping-common.h>

static inline int dma_supported(struct device *dev, u64 mask)
{
	struct dma_map_ops *ops = get_dma_ops(dev);
	return ops->dma_supported(dev, mask);
}

static inline int dma_mapping_error(struct device *dev, u64 mask)
{
	struct dma_map_ops *ops = get_special_dma_ops(dev, mask);

	debug_dma_mapping_error(dev, mask);
	return ops->mapping_error(dev, mask);
}

static inline int
dma_set_mask(struct device *dev, u64 mask)
{
	struct dma_map_ops *ops = get_dma_ops(dev);

	if(!dev->dma_mask || !dma_supported(dev, mask))
		return -EIO;

	if (ops->set_dma_mask)
		return ops->set_dma_mask(dev, mask);

	*dev->dma_mask = mask;

	return 0;
}

extern void dma_cache_sync(struct device *dev, void *vaddr, size_t size,
	       enum dma_data_direction direction);

#define dma_alloc_coherent(d,s,h,f)	dma_alloc_attrs(d,s,h,f,NULL)

static inline void *dma_alloc_attrs(struct device *dev, size_t size,
				    dma_addr_t *dma_handle, gfp_t gfp,
				    struct dma_attrs *attrs)
{
	void *ret;
	struct dma_map_ops *ops = get_dma_ops(dev);

	ret = ops->alloc(dev, size, dma_handle, gfp, attrs);

	debug_dma_alloc_coherent(dev, size, *dma_handle, ret);

	return ret;
}

#define dma_free_coherent(d,s,c,h) dma_free_attrs(d,s,c,h,NULL)

static inline void dma_free_attrs(struct device *dev, size_t size,
				  void *vaddr, dma_addr_t dma_handle,
				  struct dma_attrs *attrs)
{
	struct dma_map_ops *ops = get_dma_ops(dev);

	ops->free(dev, size, vaddr, dma_handle, attrs);

	debug_dma_free_coherent(dev, size, vaddr, dma_handle);
}


void *dma_alloc_noncoherent(struct device *dev, size_t size,
			   dma_addr_t *dma_handle, gfp_t flag);

void dma_free_noncoherent(struct device *dev, size_t size,
			 void *vaddr, dma_addr_t dma_handle);

dma_addr_t mips_unity_phys_to_dma(struct device *dev, phys_addr_t paddr);
phys_addr_t mips_unity_dma_to_phys(struct device *dev, dma_addr_t daddr);
#endif /* _ASM_DMA_MAPPING_H */
