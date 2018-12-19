/*
 * linux/sound/mips/ls2k-pcm.h -- ALSA PCM interface for the Intel PXA2xx chip
 *
 * Author:	Nicolas Pitre
 * Created:	Nov 30, 2004
 * Copyright:	MontaVista Software, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _LS1A_PCM_H
#define _LS1A_PCM_H
#include <asm/delay.h>
//#include "ls2k-pcm-private.h"
typedef struct ls2k_dma_desc {
	u32 ordered;
	u32 saddr;
	u32 daddr;
	u32 length;
	u32 step_length;
	u32 step_times;
	u32 cmd;
	u32 stats;
#ifdef DMA64
	u32 ordered_hi;
	u32 saddr_hi;
	u32 dummy[6];
#endif
} ls2k_dma_desc;

struct ls2k_runtime_data {
	int dma_ch;
	struct ls2k_pcm_dma_params *params;
	ls2k_dma_desc *dma_desc_array;
	dma_addr_t dma_desc_array_phys;

	//将要执行的dma描述符物理地址
	ls2k_dma_desc *dma_desc_ready;
	dma_addr_t dma_desc_ready_phys;

	ls2k_dma_desc *dma_position_desc;
	dma_addr_t dma_position_desc_phys;

	dma_addr_t base_phys;
	void	*base;
	dma_addr_t order_addr1_phys;
	void	*order_addr1;
	dma_addr_t order_addr2_phys;
	void	*order_addr2;
};

struct ls2k_pcm_client {
	struct ls2k_pcm_dma_params *playback_params;
	struct ls2k_pcm_dma_params *capture_params;
	int (*startup)(struct snd_pcm_substream *);
	void (*shutdown)(struct snd_pcm_substream *);
	int (*prepare)(struct snd_pcm_substream *);
};

extern int ls2k_pcm_new(struct snd_card *, struct ls2k_pcm_client *, struct snd_pcm **);


/* platform data */
extern struct snd_soc_platform_driver ls2k_soc_platform;

/*Zhuo Qixiang REGS OPRERATION*/


static inline u32 read_reg(volatile u32 * reg)
{
	return (*reg);
}
static inline void write_reg(volatile u32 * reg, u32 val)
{
	*(reg) = (val);
}

void ls2k_ac97_write (struct snd_ac97 *ac97, unsigned short reg, unsigned short val);
unsigned short ls2k_ac97_read (struct snd_ac97 *ac97, unsigned short reg);
#endif
