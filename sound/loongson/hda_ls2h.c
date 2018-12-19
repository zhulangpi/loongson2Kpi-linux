/*
 *
 *  hda_intel.c - Implementation of primary alsa driver code base
 *                for Intel HD Audio.
 *
 *  Copyright(c) 2004 Intel Corporation. All rights reserved.
 *
 *  Copyright (c) 2004 Takashi Iwai <tiwai@suse.de>
 *                     PeiSen Hou <pshou@realtek.com.tw>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the Free
 *  Software Foundation; either version 2 of the License, or (at your option)
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 *  more details.
 *
 *  You should have received a copy of the GNU General Public License along with
 *  this program; if not, write to the Free Software Foundation, Inc., 59
 *  Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 *  CONTACTS:
 *
 *  Matt Jared		matt.jared@intel.com
 *  Andy Kopp		andy.kopp@intel.com
 *  Dan Kogan		dan.d.kogan@intel.com
 *
 *  CHANGES:
 *
 *  2004.12.01	Major rewrite by tiwai, merged the work of pshou
 *
 */

#include <asm/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/reboot.h>
#include <linux/timer.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <linux/pci.h>
#include "hda_codec.h"

#ifdef CONFIG_MACH_LOONGSON2K
#include <ls2k.h>
#define LS2H_VER3 LS2K_VER3
#define LS2H_VER2 LS2K_VER2
#else
#include <ls2h/ls2h.h>
#endif
/* ls2h: enable rirb interrupt. */
#define EN_RIRBINT 0

/* ls2h: macros for convenience. */
#ifndef snd_dma_platform_data
#define snd_dma_platform_data(plat)   (&(plat)->dev)
#endif

#define platform_resource_start(dev,bar)   ((dev)->resource[(bar)].start)
#define platform_resource_end(dev,bar)     ((dev)->resource[(bar)].end)
#define platform_resource_flags(dev,bar)   ((dev)->resource[(bar)].flags)
#define platform_resource_len(dev,bar) \
        ((platform_resource_start((dev),(bar)) == 0 &&       \
          platform_resource_end((dev),(bar)) ==              \
          platform_resource_start((dev),(bar))) ? 0 :        \
                                                        \
         (platform_resource_end((dev),(bar)) -               \
          platform_resource_start((dev),(bar)) + 1))

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;
static bool enable[SNDRV_CARDS] = SNDRV_DEFAULT_ENABLE_PNP;
static char *model[SNDRV_CARDS];
static int position_fix[SNDRV_CARDS];
static int bdl_pos_adj[SNDRV_CARDS] = {[0 ... (SNDRV_CARDS - 1)] = -1 };
static int probe_mask[SNDRV_CARDS] = {[0 ... (SNDRV_CARDS - 1)] = -1 };

static int probe_only[SNDRV_CARDS];
static int single_cmd;
static int enable_msi = -1;
static u32 chip_version = LS2H_VER3;
#ifdef CONFIG_LS2H_HDA_INPUT_BEEP
static int beep_mode[SNDRV_CARDS] = {[0 ... (SNDRV_CARDS - 1)] =
	    CONFIG_LS2H_HDA_INPUT_BEEP_MODE
};
#endif

module_param_array(index, int, NULL, 0444);
MODULE_PARM_DESC(index, "Index value for Intel HD audio interface.");
module_param_array(id, charp, NULL, 0444);
MODULE_PARM_DESC(id, "ID string for Intel HD audio interface.");
module_param_array(enable, bool, NULL, 0444);
MODULE_PARM_DESC(enable, "Enable Intel HD audio interface.");
module_param_array(model, charp, NULL, 0444);
MODULE_PARM_DESC(model, "Use the given board model.");
module_param_array(position_fix, int, NULL, 0444);
MODULE_PARM_DESC(position_fix, "DMA pointer read method."
		 "(0 = auto, 1 = LPIB, 2 = POSBUF).");
module_param_array(bdl_pos_adj, int, NULL, 0644);
MODULE_PARM_DESC(bdl_pos_adj, "BDL position adjustment offset.");
module_param_array(probe_mask, int, NULL, 0444);
MODULE_PARM_DESC(probe_mask, "Bitmask to probe codecs (default = -1).");
module_param_array(probe_only, int, NULL, 0444);
MODULE_PARM_DESC(probe_only, "Only probing and no codec initialization.");
module_param(single_cmd, bool, 0444);
MODULE_PARM_DESC(single_cmd, "Use single command to communicate with codecs "
		 "(for debugging only).");
module_param(enable_msi, int, 0444);
MODULE_PARM_DESC(enable_msi, "Enable Message Signaled Interrupt (MSI)");
#ifdef CONFIG_LS2H_HDA_INPUT_BEEP
module_param_array(beep_mode, int, NULL, 0444);
MODULE_PARM_DESC(beep_mode, "Select HDA Beep registration mode "
		 "(0=off, 1=on, 2=mute switch on/off) (default=1).");
#endif

#ifdef CONFIG_SND_HDA_POWER_SAVE
static int power_save = CONFIG_LS2H_HDA_POWER_SAVE_DEFAULT;
module_param(power_save, int, 0644);
MODULE_PARM_DESC(power_save, "Automatic power-saving timeout "
		 "(in second, 0 = disable).");

/* reset the HD-audio controller in power save mode.
 * this may give more power-saving, but will take longer time to
 * wake up.
 */
static int power_save_controller = 1;
module_param(power_save_controller, bool, 0644);
MODULE_PARM_DESC(power_save_controller, "Reset controller in power save mode.");
#endif

MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("{{Intel, ICH6},"
			"{Intel, ICH6M},"
			"{Intel, ICH7},"
			"{Intel, ESB2},"
			"{Intel, ICH8}," "{Intel, ICH9}," "{Intel, ICH10}}");
MODULE_DESCRIPTION("Loongson2H HDA driver");

#ifdef CONFIG_SND_VERBOSE_PRINTK
#define SFX			/* nop */
#else
#define SFX	"hda-ls2h: "
#endif

/*
 * registers
 */
#define ICH6_REG_GCAP			0x00
#define   ICH6_GCAP_64OK	(1 << 0)	/* 64bit address support */
#define   ICH6_GCAP_NSDO	(3 << 1)	/* # of serial data out signals */
#define   ICH6_GCAP_BSS		(31 << 3)	/* # of bidirectional streams */
#define   ICH6_GCAP_ISS		(15 << 8)	/* # of input streams */
#define   ICH6_GCAP_OSS		(15 << 12)	/* # of output streams */
#define ICH6_REG_VMIN			0x02
#define ICH6_REG_VMAJ			0x03
#define ICH6_REG_OUTPAY			0x04
#define ICH6_REG_INPAY			0x06
#define ICH6_REG_GCTL			0x08
#define   ICH6_GCTL_RESET	(1 << 0)	/* controller reset */
#define   ICH6_GCTL_FCNTRL	(1 << 1)	/* flush control */
#define   ICH6_GCTL_UNSOL	(1 << 8)	/* accept unsol. response enable */
#define ICH6_REG_WAKEEN			0x0c
#define ICH6_REG_STATESTS		0x0e
#define ICH6_REG_GSTS			0x10
#define   ICH6_GSTS_FSTS	(1 << 1)	/* flush status */
#define ICH6_REG_INTCTL			0x20
#define ICH6_REG_INTSTS			0x24
#define ICH6_REG_WALLCLK		0x30	/* 24Mhz source */
#define ICH6_REG_SYNC			0x34
#define ICH6_REG_SSYNC			0x38
#define ICH6_REG_CORBLBASE		0x40
#define ICH6_REG_CORBUBASE		0x44
#define ICH6_REG_CORBWP			0x48
#define ICH6_REG_CORBRP			0x4a
#define   ICH6_CORBRP_RST	(1 << 15)	/* read pointer reset */
#define ICH6_REG_CORBCTL		0x4c
#define   ICH6_CORBCTL_RUN	(1 << 1)	/* enable DMA */
#define   ICH6_CORBCTL_CMEIE	(1 << 0)	/* enable memory error irq */
#define ICH6_REG_CORBSTS		0x4d
#define   ICH6_CORBSTS_CMEI	(1 << 0)	/* memory error indication */
#define ICH6_REG_CORBSIZE		0x4e

#define ICH6_REG_RIRBLBASE		0x50
#define ICH6_REG_RIRBUBASE		0x54
#define ICH6_REG_RIRBWP			0x58
#define   ICH6_RIRBWP_RST	(1 << 15)	/* write pointer reset */
#define ICH6_REG_RINTCNT		0x5a
#define ICH6_REG_RIRBCTL		0x5c
#define   ICH6_RBCTL_IRQ_EN	(1 << 0)	/* enable IRQ */
#define   ICH6_RBCTL_DMA_EN	(1 << 1)	/* enable DMA */
#define   ICH6_RBCTL_OVERRUN_EN	(1 << 2)	/* enable overrun irq */
#define ICH6_REG_RIRBSTS		0x5d
#define   ICH6_RBSTS_IRQ	(1 << 0)	/* response irq */
#define   ICH6_RBSTS_OVERRUN	(1 << 2)	/* overrun irq */
#define ICH6_REG_RIRBSIZE		0x5e

#define ICH6_REG_IC			0x60
#define ICH6_REG_IR			0x64
#define ICH6_REG_IRS			0x68
#define   ICH6_IRS_VALID	(1<<1)
#define   ICH6_IRS_BUSY		(1<<0)

#define ICH6_REG_DPLBASE		0x70
#define ICH6_REG_DPUBASE		0x74
#define   ICH6_DPLBASE_ENABLE	0x1	/* Enable position buffer */

/* SD offset: SDI0=0x80, SDI1=0xa0, ... SDO3=0x160 */
enum { SDI0, SDI1, SDI2, SDI3, SDO0, SDO1, SDO2, SDO3 };

/* stream register offsets from stream base */
#define ICH6_REG_SD_CTL			0x00
#define ICH6_REG_SD_STS			0x03
#define ICH6_REG_SD_LPIB		0x04
#define ICH6_REG_SD_CBL			0x08
#define ICH6_REG_SD_LVI			0x0c
#define ICH6_REG_SD_FIFOW		0x0e
#define ICH6_REG_SD_FIFOSIZE		0x10
#define ICH6_REG_SD_FORMAT		0x12
#define ICH6_REG_SD_BDLPL		0x18
#define ICH6_REG_SD_BDLPU		0x1c

/* PCI space */
#define ICH6_PCIREG_TCSEL	0x44

/*
 * other constants
 */

/* max number of SDs */
/* ICH, ATI and VIA have 4 playback and 4 capture */
#define ICH6_NUM_CAPTURE	4
#define ICH6_NUM_PLAYBACK	4

/* this number is statically defined for simplicity */
#define MAX_AZX_DEV		16

/* max number of fragments - we may use more if allocating more pages for BDL */
#define BDL_SIZE		4096
#define AZX_MAX_BDL_ENTRIES	(BDL_SIZE / 16)
#define AZX_MAX_FRAG		32
/* max buffer size - no h/w limit, you can increase as you like */
#define AZX_MAX_BUF_SIZE	(1024*1024*1024)

/* RIRB int mask: overrun[2], response[0] */
#define RIRB_INT_RESPONSE	0x01
#define RIRB_INT_OVERRUN	0x04
#define RIRB_INT_MASK		0x05

/* STATESTS int mask: S3,SD2,SD1,SD0 */
#define AZX_MAX_CODECS		8
#define AZX_DEFAULT_CODECS	4
#define STATESTS_INT_MASK	((1 << AZX_MAX_CODECS) - 1)

/* SD_CTL bits */
#define SD_CTL_STREAM_RESET	0x01	/* stream reset bit */
#define SD_CTL_DMA_START	0x02	/* stream DMA start bit */
#define SD_CTL_STRIPE		(3 << 16)	/* stripe control */
#define SD_CTL_TRAFFIC_PRIO	(1 << 18)	/* traffic priority */
#define SD_CTL_DIR		(1 << 19)	/* bi-directional stream */
#define SD_CTL_STREAM_TAG_MASK	(0xf << 20)
#define SD_CTL_STREAM_TAG_SHIFT	20

/* SD_CTL and SD_STS */
#define SD_INT_DESC_ERR		0x10	/* descriptor error interrupt */
#define SD_INT_FIFO_ERR		0x08	/* FIFO error interrupt */
#define SD_INT_COMPLETE		0x04	/* completion interrupt */
#define SD_INT_MASK		(SD_INT_DESC_ERR|SD_INT_FIFO_ERR|\
				 SD_INT_COMPLETE)

/* SD_STS */
#define SD_STS_FIFO_READY	0x20	/* FIFO ready */

/* INTCTL and INTSTS */
#define ICH6_INT_ALL_STREAM	0xff	/* all stream interrupts */
#define ICH6_INT_CTRL_EN	0x40000000	/* controller interrupt enable bit */
#define ICH6_INT_GLOBAL_EN	0x80000000	/* global interrupt enable bit */

/* below are so far hardcoded - should read registers in future */
/* ls2h */
#define ICH6_MAX_CORB_ENTRIES	256
#define ICH6_MAX_RIRB_ENTRIES	256

/* position fix mode */
enum {
	POS_FIX_AUTO,
	POS_FIX_LPIB,
	POS_FIX_POSBUF,
};

/* Defines for Intel SCH HDA snoop control */
#define INTEL_SCH_HDA_DEVC      0x78
#define INTEL_SCH_HDA_DEVC_NOSNOOP       (0x1<<11)

/* HD Audio class code */
#define PCI_CLASS_MULTIMEDIA_HD_AUDIO	0x0403

struct azx_dev {
	struct snd_dma_buffer bdl;	/* BDL buffer */
	u32 *posbuf;		/* position buffer pointer */

	unsigned int bufsize;	/* size of the play buffer in bytes */
	unsigned int period_bytes;	/* size of the period in bytes */
	unsigned int frags;	/* number for period in the play buffer */
	unsigned int fifo_size;	/* FIFO size */
	unsigned long start_wallclk;	/* start + minimum wallclk */
	unsigned long period_wallclk;	/* wallclk for period */

	void __iomem *sd_addr;	/* stream descriptor pointer */

	u32 sd_int_sta_mask;	/* stream int status mask */

	/* pcm support */
	struct snd_pcm_substream *substream;	/* assigned substream,
						 * set in PCM open
						 */
	unsigned int format_val;	/* format value to be set in the
					 * controller and the codec
					 */
	unsigned char stream_tag;	/* assigned stream */
	unsigned char index;	/* stream index */
	int device;		/* last device number assigned to */

	unsigned int opened:1;
	unsigned int running:1;
	unsigned int irq_pending:1;
	/*
	 * For VIA:
	 *  A flag to ensure DMA position is 0
	 *  when link position is not greater than FIFO size
	 */
	unsigned int insufficient:1;

	/* ls2h */
	unsigned int fix_prvpos;
};

/* CORB/RIRB */
struct azx_rb {
	u32 *buf;		/* CORB/RIRB buffer
				 * Each CORB entry is 4byte, RIRB is 8byte
				 */
	dma_addr_t addr;	/* physical address of CORB/RIRB buffer */
	/* for RIRB */
	unsigned short rp, wp;	/* read/write pointers */
	int cmds[AZX_MAX_CODECS];	/* number of pending requests */
	u32 res[AZX_MAX_CODECS];	/* last read value */
};

struct azx {
	struct snd_card *card;
	struct platform_device *dev;
	int dev_index;

	/* chip type specific */
	int driver_type;
	int playback_streams;
	int playback_index_offset;
	int capture_streams;
	int capture_index_offset;
	int num_streams;

	/* memory resources */
	unsigned long addr;
	void __iomem *remap_addr;
	int irq;

	/* locks */
	spinlock_t reg_lock;
	struct mutex open_mutex;

	/* streams (x num_streams) */
	struct azx_dev *azx_dev;

	/* PCM */
	struct snd_pcm *pcm[HDA_MAX_PCMS];

	/* HD codec */
	unsigned short codec_mask;
	int codec_probe_mask;	/* copied from probe_mask option */
	struct hda_bus *bus;
	unsigned int beep_mode;

	/* CORB/RIRB */
	struct azx_rb corb;
	struct azx_rb rirb;

	/* CORB/RIRB and position buffers */
	struct snd_dma_buffer rb;
	struct snd_dma_buffer posbuf;

	/* flags */
	int position_fix[2];	/* for both playback/capture streams */
	int poll_count;
	unsigned int running:1;
	unsigned int initialized:1;
	unsigned int single_cmd:1;
	unsigned int polling_mode:1;
	unsigned int msi:1;
	unsigned int irq_pending_warned:1;
	unsigned int probing:1;	/* codec probing phase */

	/* for debugging */
	unsigned int last_cmd[AZX_MAX_CODECS];

	/* for pending irqs */
	struct work_struct irq_pending_work;

	/* reboot notifier (for mysterious hangup problem at power-down) */
	struct notifier_block reboot_notifier;
};

/* driver types */
enum {
	AZX_DRIVER_ICH,
	AZX_NUM_DRIVERS,	/* keep this as last entry */
};

static char *driver_short_names[] = {
	[AZX_DRIVER_ICH] = "HDA Intel",
};

/*
 * macros for easy use
 */
#define azx_writel(chip,reg,value) \
	writel(value, (chip)->remap_addr + ICH6_REG_##reg)
#define azx_readl(chip,reg) \
	readl((chip)->remap_addr + ICH6_REG_##reg)

#define azx_writew(chip,reg,value) \
	ls_writew(value, (chip)->remap_addr + ICH6_REG_##reg)
#define azx_readw(chip,reg) \
	ls_readw((chip)->remap_addr + ICH6_REG_##reg)

#define azx_writeb(chip,reg,value) \
	ls_writeb(value, (chip)->remap_addr + ICH6_REG_##reg)
#define azx_readb(chip,reg) \
	ls_readb((chip)->remap_addr + ICH6_REG_##reg)


#define azx_sd_writel(dev,reg,value) \
	writel(value, (dev)->sd_addr + ICH6_REG_##reg)
#define azx_sd_readl(dev,reg) \
	readl((dev)->sd_addr + ICH6_REG_##reg)

#define azx_sd_writew(dev,reg,value) \
	ls_writew(value, (dev)->sd_addr + ICH6_REG_##reg)
#define azx_sd_readw(dev,reg) \
	ls_readw((dev)->sd_addr + ICH6_REG_##reg)

#define azx_sd_writeb(dev,reg,value) \
	ls_writeb(value, (dev)->sd_addr + ICH6_REG_##reg)
#define azx_sd_readb(dev,reg) \
	ls_readb((dev)->sd_addr + ICH6_REG_##reg)

/*
 * redefine 3A IO functions. ls2h
 */


#ifdef CONFIG_LS2H_SB
spinlock_t mailbox_lock;
static void __iomem *ls2h_mailbox_addr;
#endif

static u32(*ls_readl) (const volatile void __iomem * addr);
static void (*ls_writel) (u32 b, volatile void __iomem * addr);
static u16(*ls_readw) (const volatile void __iomem * addr);
static void (*ls_writew) (u16 b, volatile void __iomem * addr);
static u8(*ls_readb) (const volatile void __iomem * addr);
static void (*ls_writeb) (u8 b, volatile void __iomem *addr);

#ifdef CONFIG_LS2H_SB
#define WAIT_TIME 10000
/* LS3A IO functions */
/* WARNING: this rule assume ls2gq/ls3a run faster than ls2h */
#define DONE 1

static void inline wait_operation_done(void)
{
	volatile u32 *p = (volatile u32 __iomem *)ls2h_mailbox_addr;
	u32 status;
	u32 cnt = 0x8000;

	status = *(p+2);

	// 1. wait status change: 0 -> 1
	//while ( status  != DONE && cnt != 0 ) {
	while ( status  != DONE ) {
		//cnt--;
		status = *(p+2);
	}
#if 0
	// 2. wait status change: 1 -> 0
	while ( stuats  == BUSY )// busy->free
		status = *(p+2);
#endif

}

static u32 ls3a_readl(const volatile void __iomem * addr)
{
	volatile u32 *p = (volatile u32 __iomem *)ls2h_mailbox_addr;
	//u32 wait = WAIT_TIME;
//	u32 check;
	u32 val;

	unsigned long flags;

#if 0
	if (((u64) addr & 0x3) == 0) {
		return readw(addr);
	}
#endif
	//printk("hda in %s\n", __FUNCTION__);

	spin_lock_irqsave(&mailbox_lock, flags);
	//spin_lock_irq(&mailbox_lock);

	// 1. make data and address ready
	*p = ((u64) addr & 0xffff) | (7 << 16);	/* type = readl */

	// 2. wait other operation done and throw hda access request
	//while (*(p + 3) == 1);
	*(p + 3) = 1;

	// 3. wait until ls2h status free
	wait_operation_done();

	// 4. clear hda access request
	*(p + 3) = 0;

	// 5. do work left
	val = *(p + 1);

	// 6. clear ls2h status bit
	*(p + 2) = 0;

	//spin_unlock_irq(&mailbox_lock);
	spin_unlock_irqrestore(&mailbox_lock, flags);

	return val;
}

static void ls3a_writel(u32 b, volatile void __iomem * addr)
{
	//u32 check;
	//u32 wait = WAIT_TIME;
	volatile u32 *p = (volatile u32 __iomem *)ls2h_mailbox_addr;
	unsigned long flags;

#if 0
	if (((u64) addr & 0x3) == 0) {
		return writew(b, addr);
	}
#endif
	//printk("hda in %s\n", __FUNCTION__);

	spin_lock_irqsave(&mailbox_lock, flags);
	//spin_lock_irq(&mailbox_lock);
	// 1. make data and address ready
	*(p + 1) = b;
	*p = ((u64) addr & 0xffff) | (8 << 16);	/* type = writel */

	// 2. wait other operation done and throw hda access request
	//while (*(p + 3) == 1);
	*(p + 3) = 1;

	// 3. wait until ls2h status free
	wait_operation_done();

	// 4. clear hda access request
	*(p + 3) = 0;

	// 5. do work left
	//val = *(p + 1);

	// 6. clear ls2h status bit
	*(p + 2) = 0;

	spin_unlock_irqrestore(&mailbox_lock, flags);
	//spin_unlock_irq(&mailbox_lock);

}


static u16 ls3a_readw(const volatile void __iomem * addr)
{
	volatile u32 *p = (volatile u32 __iomem *)ls2h_mailbox_addr;
	//u32 wait = WAIT_TIME;
	//u32 check;
	u16 val;

	unsigned long flags;

	if (((u64) addr & 0x3) == 0) {
		return readw(addr);
	}

	//printk("hda in %s\n", __FUNCTION__);
	spin_lock_irqsave(&mailbox_lock, flags);
	//spin_lock_irq(&mailbox_lock);

	*p = ((u64) addr & 0xffff) | (5 << 16);	/* type = readw */
	*(p + 3) = 1;
	wait_operation_done();
	*(p + 3) = 0;

	val = *(const volatile u16 __force *)(p + 1);
	*(p + 2) = 0;

	spin_unlock_irqrestore(&mailbox_lock, flags);
	//spin_unlock_irq(&mailbox_lock);

	return val;
}

static void ls3a_writew(u16 b, volatile void __iomem * addr)
{
	volatile u32 *p = (volatile u32 __iomem *)ls2h_mailbox_addr;
	unsigned long flags;

	if (((u64) addr & 0x3) == 0) {
		return writew(b, addr);
	}

	//printk("hda in %s\n", __FUNCTION__);
	spin_lock_irqsave(&mailbox_lock, flags);
	//spin_lock_irq(&mailbox_lock);

	*(p + 1) = b;
	*p = ((u64) addr & 0xffff) | (2 << 16);	/* type = writew */

	*(p + 3) = 1;
	wait_operation_done();
	*(p + 3) = 0;

	*(p + 2) = 0;
	spin_unlock_irqrestore(&mailbox_lock, flags);
	//spin_unlock_irq(&mailbox_lock);
}

static u8 ls3a_readb(const volatile void __iomem * addr)
{
	u8 val;
	volatile u32 *p = (volatile u32 __iomem *)ls2h_mailbox_addr;

	unsigned long flags;

	if (((u64) addr & 0x3) == 0) {
		return readb(addr);
	}

	//printk("hda in %s\n", __FUNCTION__);
	spin_lock_irqsave(&mailbox_lock, flags);
	//spin_lock_irq(&mailbox_lock);

	*p = ((u64) addr & 0xffff) | (4 << 16);	/* type = readb */

	*(p + 3) = 1;
	wait_operation_done();
	*(p + 3) = 0;
	val = *(const volatile u8 __force *)(p + 1);

	*(p + 2) = 0;

	spin_unlock_irqrestore(&mailbox_lock, flags);
	//spin_unlock_irq(&mailbox_lock);

	return val;
}

static void ls3a_writeb(u8 b, volatile void __iomem * addr)
{
	volatile u32 *p = (volatile u32 __iomem *)ls2h_mailbox_addr;

	unsigned long flags;

	if (((u64) addr & 0x3) == 0) {
		writeb(b, addr);
		return;
	}

	//printk("hda in %s\n", __FUNCTION__);
	spin_lock_irqsave(&mailbox_lock, flags);
	//spin_lock_irq(&mailbox_lock);

	*(p + 1) = b;
	*p = ((u64) addr & 0xffff) | (1 << 16);	/* type = writeb */

	*(p + 3) = 1; //start
	wait_operation_done();
	*(p + 3) = 0;

	*(p + 2) = 0;

	spin_unlock_irqrestore(&mailbox_lock, flags);
	//spin_unlock_irq(&mailbox_lock);
}
#endif /* CONFIG_LS2H_SB */

/* for pcm support */
#define get_azx_dev(substream) (substream->runtime->private_data)

static int azx_acquire_irq(struct azx *chip, int do_disconnect);
static int azx_send_cmd(struct hda_bus *bus, unsigned int val);

/*
 * Interface for HD codec
 */

/*
 * CORB / RIRB interface
 */
static int azx_alloc_cmd_io(struct azx *chip)
{
	int err;

	/* single page (at least 4096 bytes) must suffice for both ringbuffes */
	err = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV,
				  snd_dma_platform_data(chip->dev),
				  PAGE_SIZE, &chip->rb);
	if (err < 0) {
		snd_printk(KERN_ERR SFX "cannot allocate CORB/RIRB\n");
		return err;
	}
	return 0;
}

static void azx_init_cmd_io(struct azx *chip)
{
	spin_lock_irq(&chip->reg_lock);
	/* CORB set up */
	chip->corb.addr = chip->rb.addr;
	chip->corb.buf = (u32 *) chip->rb.area;
	azx_writel(chip, CORBLBASE, (u32) chip->corb.addr);
	azx_writel(chip, CORBUBASE, upper_32_bits(chip->corb.addr));

	/* set the corb size to 256 entries (ULI requires explicitly) */
	azx_writeb(chip, CORBSIZE, 0x02);
	/* set the corb write pointer to 0 */
	azx_writew(chip, CORBWP, 0);
	/* reset the corb hw read pointer */
	azx_writew(chip, CORBRP, ICH6_CORBRP_RST);
	/* ls2h */
	azx_writew(chip, CORBRP, 0);
	/* enable corb dma */
	azx_writeb(chip, CORBCTL, ICH6_CORBCTL_RUN);
	azx_readb(chip, CORBCTL);

	/* RIRB set up */
	chip->rirb.addr = chip->rb.addr + 2048;
	chip->rirb.buf = (u32 *) (chip->rb.area + 2048);
	chip->rirb.wp = chip->rirb.rp = 0;
	memset(chip->rirb.cmds, 0, sizeof(chip->rirb.cmds));
	azx_writel(chip, RIRBLBASE, (u32) chip->rirb.addr);
	azx_writel(chip, RIRBUBASE, upper_32_bits(chip->rirb.addr));

	/* set the rirb size to 256 entries (ULI requires explicitly) */
	azx_writeb(chip, RIRBSIZE, 0x02);
	/* reset the rirb hw write pointer */
	azx_writew(chip, RIRBWP, ICH6_RIRBWP_RST);
	/* set N=1, get RIRB response interrupt for new entry */
/* ls2h */
	azx_writew(chip, RINTCNT, 1);

	/* enable rirb dma and response irq */
/* ls2h: enable rirb interrupt. */
#if EN_RIRBINT
	azx_writeb(chip, RIRBCTL, ICH6_RBCTL_DMA_EN | ICH6_RBCTL_IRQ_EN);
#else
	azx_writeb(chip, RIRBCTL, ICH6_RBCTL_DMA_EN);
	azx_readb(chip, RIRBCTL);
#endif
	spin_unlock_irq(&chip->reg_lock);
}

static void azx_free_cmd_io(struct azx *chip)
{
	spin_lock_irq(&chip->reg_lock);
	/* disable ringbuffer DMAs */
	azx_writeb(chip, RIRBCTL, 0);
	azx_writeb(chip, CORBCTL, 0);
	spin_unlock_irq(&chip->reg_lock);
}

static unsigned int azx_command_addr(u32 cmd)
{
	unsigned int addr = cmd >> 28;

	if (addr >= AZX_MAX_CODECS) {
		snd_BUG();
		addr = 0;
	}

	return addr;
}

static unsigned int azx_response_addr(u32 res)
{
	unsigned int addr = res & 0xf;

	if (addr >= AZX_MAX_CODECS) {
		addr = 0;
	}

	return addr;
}

/* send a command */
static int azx_corb_send_cmd(struct hda_bus *bus, u32 val)
{
	struct azx *chip = bus->private_data;
	unsigned int addr = azx_command_addr(val);
	unsigned int wp;

	spin_lock_irq(&chip->reg_lock);

	/* add command to corb */
	if (chip_version == LS2H_VER2) {
		wp = azx_readl(chip, CORBWP);
		wp &= 0xff;
	} else {
		wp = azx_readw(chip, CORBWP);
		wp++;
		wp %= ICH6_MAX_CORB_ENTRIES;
	}

	chip->rirb.cmds[addr]++;
	chip->corb.buf[wp] = cpu_to_le32(val);

	if (chip_version == LS2H_VER2) {
		wp++;
		wp %= ICH6_MAX_CORB_ENTRIES;
	}

	azx_writew(chip, CORBWP, wp);
	azx_readw(chip, CORBWP);
	azx_readw(chip, CORBRP);

	spin_unlock_irq(&chip->reg_lock);

	return 0;
}

#define ICH6_RIRB_EX_UNSOL_EV	(1<<4)

/* retrieve RIRB entry - called from interrupt handler */
static void azx_update_rirb(struct azx *chip)
{
	unsigned int rp, wp;
	unsigned int addr;
	u32 res, res_ex;

	if (chip_version == LS2H_VER2) {
		wp = azx_readl(chip, RIRBWP);
		wp &= 0xff;
	} else {
		wp = azx_readw(chip, RIRBWP);
	}

	if (wp == chip->rirb.wp)
		return;
	chip->rirb.wp = wp;

	while (chip->rirb.rp != wp) {
		if (chip_version != LS2H_VER2) {
			chip->rirb.rp++;
			chip->rirb.rp %= ICH6_MAX_RIRB_ENTRIES;
		}
		rp = chip->rirb.rp << 1;	/* an RIRB entry is 8-bytes */
		res_ex = le32_to_cpu(chip->rirb.buf[rp + 1]);
		res = le32_to_cpu(chip->rirb.buf[rp]);
		addr = azx_response_addr(res_ex);
		if (res_ex & ICH6_RIRB_EX_UNSOL_EV)
			ls2h_hda_queue_unsol_event(chip->bus, res, res_ex);
		else if (chip->rirb.cmds[addr]) {
			chip->rirb.res[addr] = res;
			smp_wmb();
			chip->rirb.cmds[addr]--;
		} else
			snd_printk(KERN_ERR SFX "spurious response %#x:%#x, "
				   "last cmd=%#08x\n",
				   res, res_ex, chip->last_cmd[addr]);

		if (chip_version == LS2H_VER2) {
			chip->rirb.rp++;
			chip->rirb.rp %= ICH6_MAX_RIRB_ENTRIES;
		}
	}
}

/* receive a response */
static unsigned int azx_rirb_get_response(struct hda_bus *bus,
					  unsigned int addr)
{
	struct azx *chip = bus->private_data;
	unsigned long timeout;
	int do_poll = 0;

again:
	timeout = jiffies + msecs_to_jiffies(1000);
	for (;;) {
		if (chip->polling_mode || do_poll) {
			spin_lock_irq(&chip->reg_lock);
			/* ls2h: throw away wrapped data. */
			chip->rirb.cmds[addr] %= ICH6_MAX_RIRB_ENTRIES;
			azx_update_rirb(chip);
			spin_unlock_irq(&chip->reg_lock);
		}
		if (!chip->rirb.cmds[addr]) {
			smp_rmb();
			bus->rirb_error = 0;

			if (!do_poll)
				chip->poll_count = 0;
			return chip->rirb.res[addr];	/* the last value */
		}
		if (time_after(jiffies, timeout))
			break;
		if (bus->needs_damn_long_delay)
			msleep(2);	/* temporary workaround */
		else {
			udelay(10);
			cond_resched();
		}
	}

	if (!chip->polling_mode && chip->poll_count < 2) {
		snd_printdd(SFX "azx_get_response timeout, "
			    "polling the codec once: last cmd=0x%08x\n",
			    chip->last_cmd[addr]);
		do_poll = 1;
		chip->poll_count++;
		goto again;
	}

	if (!chip->polling_mode) {
		snd_printk(KERN_WARNING SFX "azx_get_response timeout, "
			   "switching to polling mode: last cmd=0x%08x\n",
			   chip->last_cmd[addr]);
		chip->polling_mode = 1;
		goto again;
	}

	if (chip->probing) {
		/* If this critical timeout happens during the codec probing
		 * phase, this is likely an access to a non-existing codec
		 * slot.  Better to return an error and reset the system.
		 */
		return -1;
	}

	/* a fatal communication error; need either to reset or to fallback
	 * to the single_cmd mode
	 */
	bus->rirb_error = 1;
	if (bus->allow_bus_reset && !bus->response_reset && !bus->in_reset) {
		bus->response_reset = 1;
		return -1;	/* give a chance to retry */
	}

	snd_printk(KERN_ERR "hda_intel: azx_get_response timeout, "
		   "switching to single_cmd mode: last cmd=0x%08x\n",
		   chip->last_cmd[addr]);
	chip->single_cmd = 1;
	bus->response_reset = 0;
	/* release CORB/RIRB */
	azx_free_cmd_io(chip);
	/* disable unsolicited responses */
	azx_writel(chip, GCTL, azx_readl(chip, GCTL) & ~ICH6_GCTL_UNSOL);
	return -1;
}

/*
 * Use the single immediate command instead of CORB/RIRB for simplicity
 *
 * Note: according to Intel, this is not preferred use.  The command was
 *       intended for the BIOS only, and may get confused with unsolicited
 *       responses.  So, we shouldn't use it for normal operation from the
 *       driver.
 *       I left the codes, however, for debugging/testing purposes.
 */

/* receive a response */
static int azx_single_wait_for_response(struct azx *chip, unsigned int addr)
{
	int timeout = 50;

	while (timeout--) {
		/* check IRV busy bit */
		if (azx_readw(chip, IRS) & ICH6_IRS_VALID) {
			/* reuse rirb.res as the response return value */
			chip->rirb.res[addr] = azx_readl(chip, IR);
			return 0;
		}
		udelay(1);
	}
	if (printk_ratelimit())
		snd_printd(SFX "get_response timeout: IRS=0x%x\n",
			   azx_readw(chip, IRS));
	chip->rirb.res[addr] = -1;
	return -EIO;
}

/* send a command */
static int azx_single_send_cmd(struct hda_bus *bus, u32 val)
{
	struct azx *chip = bus->private_data;
	unsigned int addr = azx_command_addr(val);
	int timeout = 50;

	bus->rirb_error = 0;
	while (timeout--) {
		/* check ICB busy bit */
		if (!((azx_readw(chip, IRS) & ICH6_IRS_BUSY))) {
			/* Clear IRV valid bit */
			azx_writew(chip, IRS, azx_readw(chip, IRS) |
				   ICH6_IRS_VALID);
			azx_writel(chip, IC, val);
			azx_writew(chip, IRS, azx_readw(chip, IRS) |
				   ICH6_IRS_BUSY);
			return azx_single_wait_for_response(chip, addr);
		}
		udelay(1);
	}
	if (printk_ratelimit())
		snd_printd(SFX "send_cmd timeout: IRS=0x%x, val=0x%x\n",
			   azx_readw(chip, IRS), val);
	return -EIO;
}

/* receive a response */
static unsigned int azx_single_get_response(struct hda_bus *bus,
					    unsigned int addr)
{
	struct azx *chip = bus->private_data;
	return chip->rirb.res[addr];
}

/*
 * The below are the main callbacks from hda_codec.
 *
 * They are just the skeleton to call sub-callbacks according to the
 * current setting of chip->single_cmd.
 */

/* send a command */
static int azx_send_cmd(struct hda_bus *bus, unsigned int val)
{
	struct azx *chip = bus->private_data;
/* ls2h */
#if !EN_RIRBINT
	udelay(500);
#endif
	chip->last_cmd[azx_command_addr(val)] = val;
	if (chip->single_cmd)
		return azx_single_send_cmd(bus, val);
	else
		return azx_corb_send_cmd(bus, val);
}

/* get a response */
static unsigned int azx_get_response(struct hda_bus *bus, unsigned int addr)
{
	struct azx *chip = bus->private_data;
	if (chip->single_cmd)
		return azx_single_get_response(bus, addr);
	else
		return azx_rirb_get_response(bus, addr);
}

#ifdef CONFIG_SND_HDA_POWER_SAVE
static void azx_power_notify(struct hda_bus *bus);
#endif

/* reset codec link */
static int azx_reset(struct azx *chip, int full_reset)
{
	int count;
	int saved_code[8];
	unsigned long flags;
	int i;

	if (!full_reset)
		goto __skip;

	/* LS2H hda pos buffer will work soon after reset and before
	 * DPLBASE be set. Dma will damanged memory 0xffffffff80000000+n*ch,
	 * ch=4-7. Here we save code before reset deassert and recover code
	 * after DPLBASE is set.
	 */
	local_irq_save(flags);
	for (i = 0; i < 8; i++)
		saved_code[i] = *(volatile int *)(0xffffffff80000000 + i * 8);

	/* clear STATESTS */
	azx_writeb(chip, STATESTS, STATESTS_INT_MASK);

	/* reset controller */
	azx_writel(chip, GCTL, azx_readl(chip, GCTL) & ~ICH6_GCTL_RESET);

	count = 50;
	while (azx_readb(chip, GCTL) && --count)
		msleep(1);

	/* delay for >= 100us for codec PLL to settle per spec
	 * Rev 0.9 section 5.5.1
	 */
	msleep(1);

	count = 50;

	/* Bring controller out of reset */
	azx_writeb(chip, GCTL, azx_readb(chip, GCTL) | ICH6_GCTL_RESET);

	do azx_writel(chip, DPLBASE, (u32) chip->posbuf.addr);
	while (!azx_readb(chip, GCTL) && --count);

	/* Brent Chartrand said to wait >= 540us for codecs to initialize */
	msleep(1);
	for (i = 0; i < 8; i++) {
		if( *(volatile int *)(0xffffffff80000000 + i*8) != saved_code[i])
			*(volatile int *)(0xffffffff80000000 + i*8) = saved_code[i];
	}
	local_irq_restore(flags);

__skip:
	/* check to see if controller is ready */
	if (!azx_readb(chip, GCTL)) {
		snd_printd(SFX "azx_reset: controller not ready!\n");
		return -EBUSY;
	}

	/* Accept unsolicited responses */
	if (!chip->single_cmd)
		azx_writel(chip, GCTL, azx_readl(chip, GCTL) &
			   ~ICH6_GCTL_UNSOL);

	/* detect codecs */
	if (!chip->codec_mask) {
		chip->codec_mask = azx_readw(chip, STATESTS);
		chip->codec_mask &= 0x1;
		snd_printdd(SFX "codec_mask = 0x%x\n", chip->codec_mask);
	}

	return 0;
}

/*
 * Lowlevel interface
 */

/* enable interrupts */
static void azx_int_enable(struct azx *chip)
{
	/* enable controller CIE and GIE */
	azx_writel(chip, INTCTL, azx_readl(chip, INTCTL) |
		   ICH6_INT_CTRL_EN | ICH6_INT_GLOBAL_EN);
}

/* disable interrupts */
static void azx_int_disable(struct azx *chip)
{
	int i;

	/* disable interrupts in stream descriptor */
	for (i = 0; i < chip->num_streams; i++) {
		struct azx_dev *azx_dev = &chip->azx_dev[i];
		azx_sd_writeb(azx_dev, SD_CTL,
			      azx_sd_readb(azx_dev, SD_CTL) & ~SD_INT_MASK);
	}

	/* disable SIE for all streams */
	azx_writeb(chip, INTCTL, 0);

	/* disable controller CIE and GIE */
	azx_writel(chip, INTCTL, azx_readl(chip, INTCTL) &
		   ~(ICH6_INT_CTRL_EN | ICH6_INT_GLOBAL_EN));
}

/* clear interrupts */
static void azx_int_clear(struct azx *chip)
{
	int i;

	/* clear stream status */
	for (i = 0; i < chip->num_streams; i++) {
		struct azx_dev *azx_dev = &chip->azx_dev[i];
		azx_sd_writeb(azx_dev, SD_STS, azx_sd_readb(azx_dev, SD_STS));
	}

	/* clear STATESTS */
	azx_writeb(chip, STATESTS, STATESTS_INT_MASK);

	/* clear rirb status */
	azx_writeb(chip, RIRBSTS, azx_readb(chip, RIRBSTS) & RIRB_INT_MASK);

	/* clear int status */
	azx_writel(chip, INTSTS, ICH6_INT_CTRL_EN | ICH6_INT_ALL_STREAM);
}

/* start a stream */
static void azx_stream_start(struct azx *chip, struct azx_dev *azx_dev)
{
	/*
	 * Before stream start, initialize parameter
	 */
	azx_dev->insufficient = 1;

	/* enable SIE */
	azx_writel(chip, INTCTL,
		   azx_readl(chip, INTCTL) | (1 << azx_dev->index));
	/* set DMA start and interrupt mask */
	/* ls2h: write byte will flush neighbour bytes to 0. */
	azx_sd_writel(azx_dev, SD_CTL, azx_sd_readl(azx_dev, SD_CTL) |
		      SD_CTL_DMA_START | SD_INT_MASK);
	if (chip_version == LS2H_VER3) {
		azx_sd_writel(azx_dev, SD_CTL,
				(azx_sd_readl(azx_dev, SD_CTL) & ~SD_CTL_STREAM_TAG_MASK)
				| (azx_dev->stream_tag << SD_CTL_STREAM_TAG_SHIFT));
	}
}

static unsigned int azx_get_position_org(struct azx *chip, struct azx_dev *azx_dev);

/* stop DMA */
static void azx_stream_clear(struct azx *chip, struct azx_dev *azx_dev)
{
	azx_sd_writel(azx_dev, SD_CTL, azx_sd_readl(azx_dev, SD_CTL) &
		      ~(SD_CTL_DMA_START | SD_INT_MASK));
	azx_sd_writeb(azx_dev, SD_STS, azx_sd_readb(azx_dev, SD_STS));
	azx_dev->fix_prvpos = azx_get_position_org(chip, azx_dev);
}

/* stop a stream */
static void azx_stream_stop(struct azx *chip, struct azx_dev *azx_dev)
{
	azx_stream_clear(chip, azx_dev);
	/* disable SIE */
	azx_writel(chip, INTCTL,
		   azx_readl(chip, INTCTL) & ~(1 << azx_dev->index));
}

/*
 * reset and start the controller registers
 */
static void azx_init_chip(struct azx *chip, int full_reset)
{
	if (chip->initialized)
		return;

	/* reset controller */
	azx_reset(chip, full_reset);

	/* initialize interrupts */
	azx_int_clear(chip);
	azx_int_enable(chip);

	/* initialize the codec command I/O */
	if (!chip->single_cmd)
		azx_init_cmd_io(chip);

	/* program the position buffer */
	azx_writel(chip, DPLBASE, (u32) chip->posbuf.addr);
	azx_writel(chip, DPUBASE, upper_32_bits(chip->posbuf.addr));

	chip->initialized = 1;
}

static int azx_position_ok(struct azx *chip, struct azx_dev *azx_dev);
/* ls2h */
static unsigned int azx_get_position(struct azx *chip, struct azx_dev *azx_dev);

/*
 * interrupt handler
 */
static irqreturn_t azx_interrupt(int irq, void *dev_id)
{
	struct azx *chip = dev_id;
	struct azx_dev *azx_dev;
	u32 status;
	u8 sd_status;
	int i, ok;

	spin_lock(&chip->reg_lock);

/* ls2h */
	status = 0;
	for (i = 0; i < chip->num_streams; i++) {
		azx_dev = &chip->azx_dev[i];
		status |= (azx_sd_readb(azx_dev, SD_STS) & SD_INT_MASK) ?
		    (1 << i) : 0;
	}
/* ls2h: enable rirb interrupt. */
#if EN_RIRBINT
	status |= (azx_readb(chip, RIRBSTS) & RIRB_INT_MASK) ? (1 << 30) : 0;
#endif
	status |= (status & ~0) ? (1 << 31) : 0;

/* ls2h */
	if (status == 0) {
		spin_unlock(&chip->reg_lock);
		return IRQ_NONE;
	}

	for (i = 0; i < chip->num_streams; i++) {
		azx_dev = &chip->azx_dev[i];
		if (status & azx_dev->sd_int_sta_mask) {
			sd_status = azx_sd_readb(azx_dev, SD_STS);
			azx_sd_writeb(azx_dev, SD_STS, sd_status);
			azx_sd_readb(azx_dev, SD_STS);
			if (!azx_dev->substream || !azx_dev->running ||
			    !(sd_status & SD_INT_COMPLETE))
				continue;
			/* check whether this IRQ is really acceptable */
			ok = azx_position_ok(chip, azx_dev);
			if (ok == 1) {
				azx_dev->irq_pending = 0;
				spin_unlock(&chip->reg_lock);
				snd_pcm_period_elapsed(azx_dev->substream);
				spin_lock(&chip->reg_lock);
			} else if (ok == 0 && chip->bus && chip->bus->workq) {
				/* bogus IRQ, process it later */
				azx_dev->irq_pending = 1;
				queue_work(chip->bus->workq,
					   &chip->irq_pending_work);
			}
		}
	}

	/* clear rirb int */
	status = azx_readb(chip, RIRBSTS);
	if (status & RIRB_INT_MASK) {
		if (status & RIRB_INT_RESPONSE) {
			azx_update_rirb(chip);
		}
		azx_writeb(chip, RIRBSTS, status & RIRB_INT_MASK);
	}
#if 0
	/* clear state status int */
	if (azx_readb(chip, STATESTS) & 0x04)
		azx_writeb(chip, STATESTS, 0x04);
#endif
	spin_unlock(&chip->reg_lock);

	return IRQ_HANDLED;
}

/*
 * set up a BDL entry
 */
static int setup_bdle(struct snd_pcm_substream *substream,
		      struct azx_dev *azx_dev, u32 ** bdlp,
		      int ofs, int size, int with_ioc)
{
	u32 *bdl = *bdlp;

	while (size > 0) {
		dma_addr_t addr;
		int chunk;

		if (azx_dev->frags >= AZX_MAX_BDL_ENTRIES)
			return -EINVAL;

		addr = snd_pcm_sgbuf_get_addr(substream, ofs);
		/* program the address field of the BDL entry */
		bdl[0] = cpu_to_le32((u32) addr);
		bdl[1] = cpu_to_le32(upper_32_bits(addr));
		/* program the size field of the BDL entry */
		chunk = snd_pcm_sgbuf_get_chunk_size(substream, ofs, size);
		bdl[2] = cpu_to_le32(chunk);
		/* program the IOC to enable interrupt
		 * only when the whole fragment is processed
		 */
		size -= chunk;
		bdl[3] = (size || !with_ioc) ? 0 : cpu_to_le32(0x01);
		bdl += 4;
		azx_dev->frags++;
		ofs += chunk;
	}
	*bdlp = bdl;
	return ofs;
}

/*
 * set up BDL entries
 */
static int azx_setup_periods(struct azx *chip,
			     struct snd_pcm_substream *substream,
			     struct azx_dev *azx_dev)
{
	u32 *bdl;
	int i, ofs, periods, period_bytes;
	int pos_adj;

	/* reset BDL address */
	azx_sd_writel(azx_dev, SD_BDLPL, 0);
	azx_sd_writel(azx_dev, SD_BDLPU, 0);

	period_bytes = azx_dev->period_bytes;
	periods = azx_dev->bufsize / period_bytes;

	/* program the initial BDL entries */
	bdl = (u32 *) azx_dev->bdl.area;
	ofs = 0;
	azx_dev->frags = 0;
	pos_adj = bdl_pos_adj[chip->dev_index];
/* ls2h */
	pos_adj = 0;
	if (pos_adj > 0) {
		struct snd_pcm_runtime *runtime = substream->runtime;
		int pos_align = pos_adj;
		pos_adj = (pos_adj * runtime->rate + 47999) / 48000;
		if (!pos_adj)
			pos_adj = pos_align;
		else
			pos_adj = ((pos_adj + pos_align - 1) / pos_align) *
			    pos_align;
		pos_adj = frames_to_bytes(runtime, pos_adj);
		if (pos_adj >= period_bytes) {
			snd_printk(KERN_WARNING SFX "Too big adjustment %d\n",
				   bdl_pos_adj[chip->dev_index]);
			pos_adj = 0;
		} else {
			ofs = setup_bdle(substream, azx_dev,
					 &bdl, ofs, pos_adj, 1);
			if (ofs < 0)
				goto error;
		}
	} else
		pos_adj = 0;
	for (i = 0; i < periods; i++) {
		if (i == periods - 1 && pos_adj)
			ofs = setup_bdle(substream, azx_dev, &bdl, ofs,
					 period_bytes - pos_adj, 0);
		else
			ofs = setup_bdle(substream, azx_dev, &bdl, ofs,
					 period_bytes, 1);
		if (ofs < 0)
			goto error;
	}

	return 0;

error:
	snd_printk(KERN_ERR SFX "Too many BDL entries: buffer=%d, period=%d\n",
		   azx_dev->bufsize, period_bytes);
	return -EINVAL;
}

/* reset stream */
static void azx_stream_reset(struct azx *chip, struct azx_dev *azx_dev)
{
#ifdef AZX_STREAM_NEED_RESET
	unsigned char val;
	int timeout;

	azx_stream_clear(chip, azx_dev);

	azx_sd_writeb(azx_dev, SD_CTL, azx_sd_readb(azx_dev, SD_CTL) |
		      SD_CTL_STREAM_RESET);
	udelay(3);
	timeout = 300;
	while (!((val = azx_sd_readb(azx_dev, SD_CTL)) & SD_CTL_STREAM_RESET) &&
	       --timeout) ;
	val &= ~SD_CTL_STREAM_RESET;
	azx_sd_writeb(azx_dev, SD_CTL, val);
	udelay(3);

	timeout = 300;
	/* waiting for hardware to report that the stream is out of reset */
	while (((val = azx_sd_readb(azx_dev, SD_CTL)) & SD_CTL_STREAM_RESET) &&
	       --timeout) ;
#endif
	/* reset first position - may not be synced with hw at this time */
	*azx_dev->posbuf = 0;
}

/*
 * set up the SD for streaming
 */
static int azx_setup_controller(struct azx *chip, struct azx_dev *azx_dev)
{
	/* make sure the run bit is zero for SD */
	azx_stream_clear(chip, azx_dev);
	/* program the stream_tag */
	if (chip_version == LS2H_VER2) {
		azx_sd_writel(azx_dev, SD_CTL,
				(azx_sd_readl(azx_dev, SD_CTL) & ~SD_CTL_STREAM_TAG_MASK)
				| (azx_dev->stream_tag << SD_CTL_STREAM_TAG_SHIFT));
	}
	else
	{
		azx_sd_writel(azx_dev, SD_CTL,
				(azx_sd_readl(azx_dev, SD_CTL) & ~SD_CTL_STREAM_TAG_MASK));
	}

	/* program the length of samples in cyclic buffer */
	/*fix hardware pointer, different form intel hda
	play: bufsize-64
	record: bufsize-16
	*/
	if(azx_dev->substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	 azx_sd_writel(azx_dev, SD_CBL, azx_dev->bufsize-64);
	else
	 azx_sd_writel(azx_dev, SD_CBL, azx_dev->bufsize-16);

	/* program the stream format */
	/* this value needs to be the same as the one programmed */
	azx_sd_writew(azx_dev, SD_FORMAT, azx_dev->format_val);

	/* program the stream LVI (last valid index) of the BDL */
	azx_sd_writew(azx_dev, SD_LVI, azx_dev->frags - 1);

	/* program the BDL address */
	/* lower BDL address */
	azx_sd_writel(azx_dev, SD_BDLPL, (u32) azx_dev->bdl.addr);
	/* upper BDL address */
	azx_sd_writel(azx_dev, SD_BDLPU, upper_32_bits(azx_dev->bdl.addr));

	/* enable the position buffer */
	if (chip->position_fix[0] != POS_FIX_LPIB ||
	    chip->position_fix[1] != POS_FIX_LPIB) {
		if (!(azx_readl(chip, DPLBASE) & ICH6_DPLBASE_ENABLE))
			azx_writel(chip, DPLBASE,
				   (u32) chip->posbuf.
				   addr | ICH6_DPLBASE_ENABLE);
	}

	/* set the interrupt enable bits in the descriptor control register */
	azx_sd_writel(azx_dev, SD_CTL,
		      azx_sd_readl(azx_dev, SD_CTL) | SD_INT_MASK);

	return 0;
}

/*
 * Probe the given codec address
 */
static int probe_codec(struct azx *chip, int addr)
{
	unsigned int cmd = (addr << 28) | (AC_NODE_ROOT << 20) |
	    (AC_VERB_PARAMETERS << 8) | AC_PAR_VENDOR_ID;
	unsigned int res;

	mutex_lock(&chip->bus->cmd_mutex);
	chip->probing = 1;
	azx_send_cmd(chip->bus, cmd);
	res = azx_get_response(chip->bus, addr);
	chip->probing = 0;
	mutex_unlock(&chip->bus->cmd_mutex);
	if (res == -1)
		return -EIO;
	snd_printdd(SFX "codec #%d probed OK\n", addr);
	return 0;
}

static int azx_attach_pcm_stream(struct hda_bus *bus, struct hda_codec *codec,
				 struct hda_pcm *cpcm);
static void azx_stop_chip(struct azx *chip);

static void azx_bus_reset(struct hda_bus *bus)
{
	struct azx *chip = bus->private_data;

	bus->in_reset = 1;
	azx_stop_chip(chip);
	azx_init_chip(chip, 1);
#ifdef CONFIG_PM
	if (chip->initialized) {
		int i;

		for (i = 0; i < HDA_MAX_PCMS; i++)
			snd_pcm_suspend_all(chip->pcm[i]);
		ls2h_hda_suspend(chip->bus);
		ls2h_hda_resume(chip->bus);
	}
#endif
	bus->in_reset = 0;
}

/*
 * Codec initialization
 */

/* number of codec slots for each chipset: 0 = default slots (i.e. 4) */
static unsigned int azx_max_codecs[AZX_NUM_DRIVERS] = {
};

static int azx_codec_create(struct azx *chip, const char *model)
{
	struct hda_bus_template bus_temp;
	int c, codecs, err;
	int max_slots;

	memset(&bus_temp, 0, sizeof(bus_temp));
	bus_temp.private_data = chip;
	bus_temp.modelname = model;
	bus_temp.dev = chip->dev;
	bus_temp.ops.command = azx_send_cmd;
	bus_temp.ops.get_response = azx_get_response;
	bus_temp.ops.attach_pcm = azx_attach_pcm_stream;
	bus_temp.ops.bus_reset = azx_bus_reset;
#ifdef CONFIG_SND_HDA_POWER_SAVE
	bus_temp.power_save = &power_save;
	bus_temp.ops.pm_notify = azx_power_notify;
#endif

	err = ls2h_hda_bus_new(chip->card, &bus_temp, &chip->bus);
	if (err < 0)
		return err;

	codecs = 0;
	max_slots = azx_max_codecs[chip->driver_type];
	if (!max_slots)
		max_slots = AZX_DEFAULT_CODECS;

	/* First try to probe all given codec slots */
	for (c = 0; c < max_slots; c++) {
		if ((chip->codec_mask & (1 << c)) & chip->codec_probe_mask) {
			if (probe_codec(chip, c) < 0) {
				/* Some BIOSen give you wrong codec addresses
				 * that don't exist
				 */
				snd_printk(KERN_WARNING SFX
					   "Codec #%d probe error; "
					   "disabling it...\n", c);
				chip->codec_mask &= ~(1 << c);
				/* More badly, accessing to a non-existing
				 * codec often screws up the controller chip,
				 * and disturbs the further communications.
				 * Thus if an error occurs during probing,
				 * better to reset the controller chip to
				 * get back to the sanity state.
				 */
				azx_stop_chip(chip);
				azx_init_chip(chip, 1);
			}
		}
	}

	/* Then create codec instances */
	for (c = 0; c < max_slots; c++) {
		if ((chip->codec_mask & (1 << c)) & chip->codec_probe_mask) {
			struct hda_codec *codec;
			err = ls2h_hda_codec_new(chip->bus, c, &codec);
			if (err < 0)
				continue;
			codec->beep_mode = chip->beep_mode;
			codecs++;
		}
	}
	if (!codecs) {
		snd_printk(KERN_ERR SFX "no codecs initialized\n");
		return -ENXIO;
	}
	return 0;
}

/* configure each codec instance */
static int azx_codec_configure(struct azx *chip)
{
	struct hda_codec *codec;
	list_for_each_entry(codec, &chip->bus->codec_list, list) {
		ls2h_hda_codec_configure(codec);
	}
	return 0;
}

/*
 * PCM support
 */

/* assign a stream for the PCM */
static inline struct azx_dev *azx_assign_device(struct azx *chip,
						struct snd_pcm_substream
						*substream)
{
	int dev, i, nums;
	struct azx_dev *res = NULL;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		dev = chip->playback_index_offset;
		nums = chip->playback_streams;
	} else {
		dev = chip->capture_index_offset;
		nums = chip->capture_streams;
	}
	for (i = 0; i < nums; i++, dev++)
		if (!chip->azx_dev[dev].opened) {
			res = &chip->azx_dev[dev];
			if (res->device == substream->pcm->device)
				break;
		}
	if (res) {
		res->opened = 1;
		res->device = substream->pcm->device;
	}
	return res;
}

/* release the assigned stream */
static inline void azx_release_device(struct azx_dev *azx_dev)
{
	azx_dev->opened = 0;
}

static struct snd_pcm_hardware azx_pcm_hw = {
	.info = (SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_BLOCK_TRANSFER | SNDRV_PCM_INFO_MMAP_VALID |
		 /* No full-resume yet implemented */
		 /* SNDRV_PCM_INFO_RESUME | */
		 SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_SYNC_START),
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_48000,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = AZX_MAX_BUF_SIZE,
	.period_bytes_min = 128,
	.period_bytes_max = AZX_MAX_BUF_SIZE / 2,
	.periods_min = 2,
	.periods_max = AZX_MAX_FRAG,
	.fifo_size = 0,
};

struct azx_pcm {
	struct azx *chip;
	struct hda_codec *codec;
	struct hda_pcm_stream *hinfo[2];
};

static int azx_pcm_open(struct snd_pcm_substream *substream)
{
	struct azx_pcm *apcm = snd_pcm_substream_chip(substream);
	struct hda_pcm_stream *hinfo = apcm->hinfo[substream->stream];
	struct azx *chip = apcm->chip;
	struct azx_dev *azx_dev;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long flags;
	int err;

	mutex_lock(&chip->open_mutex);
	azx_dev = azx_assign_device(chip, substream);
	if (azx_dev == NULL) {
		mutex_unlock(&chip->open_mutex);
		return -EBUSY;
	}
	runtime->hw = azx_pcm_hw;
	runtime->hw.channels_min = hinfo->channels_min;
	runtime->hw.channels_max = hinfo->channels_max;
	runtime->hw.formats = hinfo->formats;
	runtime->hw.rates = hinfo->rates;
	snd_pcm_limit_hw_rates(runtime);
	snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
				   128);
	snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
				   128);
	ls2h_hda_power_up(apcm->codec);
	err = hinfo->ops.open(hinfo, apcm->codec, substream);
	if (err < 0) {
		azx_release_device(azx_dev);
		ls2h_hda_power_down(apcm->codec);
		mutex_unlock(&chip->open_mutex);
		return err;
	}
	snd_pcm_limit_hw_rates(runtime);
	/* sanity check */
	if (snd_BUG_ON(!runtime->hw.channels_min) ||
	    snd_BUG_ON(!runtime->hw.channels_max) ||
	    snd_BUG_ON(!runtime->hw.formats) ||
	    snd_BUG_ON(!runtime->hw.rates)) {
		azx_release_device(azx_dev);
		hinfo->ops.close(hinfo, apcm->codec, substream);
		ls2h_hda_power_down(apcm->codec);
		mutex_unlock(&chip->open_mutex);
		return -EINVAL;
	}
	spin_lock_irqsave(&chip->reg_lock, flags);
	azx_dev->substream = substream;
	azx_dev->running = 0;
	spin_unlock_irqrestore(&chip->reg_lock, flags);

	runtime->private_data = azx_dev;
	snd_pcm_set_sync(substream);
	mutex_unlock(&chip->open_mutex);
	return 0;
}

static int azx_pcm_close(struct snd_pcm_substream *substream)
{
	struct azx_pcm *apcm = snd_pcm_substream_chip(substream);
	struct hda_pcm_stream *hinfo = apcm->hinfo[substream->stream];
	struct azx *chip = apcm->chip;
	struct azx_dev *azx_dev = get_azx_dev(substream);
	unsigned long flags;

	mutex_lock(&chip->open_mutex);
	spin_lock_irqsave(&chip->reg_lock, flags);
	azx_dev->substream = NULL;
	azx_dev->running = 0;
	spin_unlock_irqrestore(&chip->reg_lock, flags);
	azx_release_device(azx_dev);
	hinfo->ops.close(hinfo, apcm->codec, substream);
	ls2h_hda_power_down(apcm->codec);
	mutex_unlock(&chip->open_mutex);
	return 0;
}

static int azx_pcm_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *hw_params)
{
	struct azx_dev *azx_dev = get_azx_dev(substream);

	azx_dev->bufsize = 0;
	azx_dev->period_bytes = 0;
	azx_dev->format_val = 0;
	return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
}

static int azx_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct azx_pcm *apcm = snd_pcm_substream_chip(substream);
	struct azx_dev *azx_dev = get_azx_dev(substream);
	struct hda_pcm_stream *hinfo = apcm->hinfo[substream->stream];

	/* reset BDL address */
	azx_sd_writel(azx_dev, SD_BDLPL, 0);
	azx_sd_writel(azx_dev, SD_BDLPU, 0);
	azx_sd_writel(azx_dev, SD_CTL, 0);
	azx_dev->bufsize = 0;
	azx_dev->period_bytes = 0;
	azx_dev->format_val = 0;

	ls2h_hda_codec_cleanup(apcm->codec, hinfo, substream);

	return snd_pcm_lib_free_pages(substream);
}

static int azx_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct azx_pcm *apcm = snd_pcm_substream_chip(substream);
	struct azx *chip = apcm->chip;
	struct azx_dev *azx_dev = get_azx_dev(substream);
	struct hda_pcm_stream *hinfo = apcm->hinfo[substream->stream];
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int bufsize, period_bytes, format_val, stream_tag;
	int err;

	azx_stream_reset(chip, azx_dev);
	format_val = ls2h_hda_calc_stream_format(runtime->rate,
						 runtime->channels,
						 runtime->format,
						 hinfo->maxbps,
						 apcm->codec->spdif_ctls);
	if (!format_val) {
		snd_printk(KERN_ERR SFX
			   "invalid format_val, rate=%d, ch=%d, format=%d\n",
			   runtime->rate, runtime->channels, runtime->format);
		return -EINVAL;
	}

	bufsize = snd_pcm_lib_buffer_bytes(substream);
	period_bytes = snd_pcm_lib_period_bytes(substream);

	snd_printdd(SFX "azx_pcm_prepare: bufsize=0x%x, format=0x%x\n",
		    bufsize, format_val);

	if (bufsize != azx_dev->bufsize ||
	    period_bytes != azx_dev->period_bytes ||
	    format_val != azx_dev->format_val) {
		azx_dev->bufsize = bufsize;
		azx_dev->period_bytes = period_bytes;
		azx_dev->format_val = format_val;
		err = azx_setup_periods(chip, substream, azx_dev);
		if (err < 0)
			return err;
	}

	/* wallclk has 24Mhz clock source */
	azx_dev->period_wallclk = (((runtime->period_size * 24000) /
				    runtime->rate) * 1000);
	azx_setup_controller(chip, azx_dev);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		azx_dev->fifo_size = azx_sd_readw(azx_dev, SD_FIFOSIZE) + 1;
	else
		azx_dev->fifo_size = 0;

	stream_tag = azx_dev->stream_tag;

	return ls2h_hda_codec_prepare(apcm->codec, hinfo, stream_tag,
				      azx_dev->format_val, substream);
}

static int azx_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct azx_pcm *apcm = snd_pcm_substream_chip(substream);
	struct azx *chip = apcm->chip;
	struct azx_dev *azx_dev;
	struct snd_pcm_substream *s;
	int rstart = 0, start, nsync = 0, sbits = 0;
	int nwait, timeout;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		rstart = 1;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		start = 1;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
		start = 0;
		break;
	default:
		return -EINVAL;
	}

	snd_pcm_group_for_each_entry(s, substream) {
		if (s->pcm->card != substream->pcm->card)
			continue;
		azx_dev = get_azx_dev(s);
		sbits |= 1 << azx_dev->index;
		nsync++;
		snd_pcm_trigger_done(s, substream);
	}

	spin_lock(&chip->reg_lock);

	if (chip_version == LS2H_VER2) {
		if (nsync > 1) {
			/* first, set SYNC bits of corresponding streams */
			azx_writel(chip, SYNC, azx_readl(chip, SYNC) | sbits);
		}
	}
	else
	{
		azx_writel(chip, SSYNC, azx_readl(chip, SSYNC) | sbits);
	}

	snd_pcm_group_for_each_entry(s, substream) {
		if (s->pcm->card != substream->pcm->card)
			continue;
		azx_dev = get_azx_dev(s);
		if (start) {
			azx_dev->start_wallclk = azx_readl(chip, WALLCLK);
/* ls2h */
#if 0
			if (!rstart)
				azx_dev->start_wallclk -=
				    azx_dev->period_wallclk;
#endif
			azx_stream_start(chip, azx_dev);
		} else {
			azx_stream_stop(chip, azx_dev);
		}
		azx_dev->running = start;
	}
	spin_unlock(&chip->reg_lock);
	if (start) {
		if (chip_version == LS2H_VER2) {
			if (nsync == 1)
				return 0;

		}
		/* wait until all FIFOs get ready */
		for (timeout = 5000; timeout; timeout--) {
			nwait = 0;
			snd_pcm_group_for_each_entry(s, substream) {
				if (s->pcm->card != substream->pcm->card)
					continue;
				azx_dev = get_azx_dev(s);
				if (!(azx_sd_readb(azx_dev, SD_STS) &
				      SD_STS_FIFO_READY))
					nwait++;
			}
			if (!nwait)
				break;
			cpu_relax();
		}
	} else {
		/* wait until all RUN bits are cleared */
		for (timeout = 5000; timeout; timeout--) {
			nwait = 0;
			snd_pcm_group_for_each_entry(s, substream) {
				if (s->pcm->card != substream->pcm->card)
					continue;
				azx_dev = get_azx_dev(s);
				if (azx_sd_readb(azx_dev, SD_CTL) &
				    SD_CTL_DMA_START)
					nwait++;
			}
			if (!nwait)
				break;
			cpu_relax();
		}
	}

	if (chip_version == LS2H_VER2) {
		if (nsync > 1) {
			spin_lock(&chip->reg_lock);
			/* reset SYNC bits */
			azx_writel(chip, SYNC, azx_readl(chip, SYNC) & ~sbits);
			spin_unlock(&chip->reg_lock);
		}
	}
	else
	{
		spin_lock(&chip->reg_lock);
		/* reset SYNC bits */
		azx_writel(chip, SSYNC, azx_readl(chip, SSYNC) & ~sbits);
		spin_unlock(&chip->reg_lock);

	}

	return 0;
}

static unsigned int azx_get_position_org(struct azx *chip, struct azx_dev *azx_dev)
{
	unsigned int pos;
	int stream;

	if(!azx_dev->substream)
		return 0;

	stream = azx_dev->substream->stream;

	switch (chip->position_fix[stream]) {
	case POS_FIX_LPIB:
		/* read LPIB */
		/* pos = azx_sd_readl(azx_dev, SD_LPIB); */
		pos = ls_readl(azx_dev->sd_addr + ICH6_REG_SD_LPIB);
		break;
	default:
		/* use the position buffer */
		pos = le32_to_cpu(*azx_dev->posbuf);
	}

	return pos;
}

/*
 * fix ls2h pos not reset. when stream clear, dma reset.
 * but stream reset can not reset pos (LPIB).
 * here we fix it but minus pos at last stop position.
 * when LPIB exceed bufsize, it will increase until round back and equal to bufsize.
 */
static unsigned int azx_get_position(struct azx *chip, struct azx_dev *azx_dev)
{
	unsigned int pos = azx_get_position_org(chip, azx_dev);

	if (pos >= azx_dev->fix_prvpos) {
		pos = pos - azx_dev->fix_prvpos;
		pos %= azx_dev->bufsize;
	} else {
		if (azx_dev->fix_prvpos > azx_dev->bufsize)
			pos = (0x100000000ULL + pos-azx_dev->fix_prvpos)
				% azx_dev->bufsize;
		else
			pos = pos + azx_dev->bufsize - azx_dev->fix_prvpos;
	}

	return pos;
}

snd_pcm_uframes_t azx_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct azx_pcm *apcm;
	struct azx *chip;
	struct azx_dev *azx_dev;

	unsigned int pos;

	apcm = snd_pcm_substream_chip(substream);
	chip = apcm->chip;
	azx_dev = get_azx_dev(substream);

	pos = azx_get_position(chip, azx_dev);

	return bytes_to_frames(substream->runtime,pos);


}

/*
 * Check whether the current DMA position is acceptable for updating
 * periods.  Returns non-zero if it's OK.
 *
 * Many HD-audio controllers appear pretty inaccurate about
 * the update-IRQ timing.  The IRQ is issued before actually the
 * data is processed.  So, we need to process it afterwords in a
 * workqueue.
 */
static int azx_position_ok(struct azx *chip, struct azx_dev *azx_dev)
{
/*	u32 wallclk; */
	unsigned int pos;
	int stream;

/* ls2h: not implemented */
#if 0
	wallclk = azx_readl(chip, WALLCLK) - azx_dev->start_wallclk;
	if (wallclk < (azx_dev->period_wallclk * 2) / 3)
		return -1;	/* bogus (too early) interrupt */
#endif

	stream = azx_dev->substream->stream;
	pos = azx_get_position(chip, azx_dev);
	if (chip->position_fix[stream] == POS_FIX_AUTO) {
		if (!pos) {
			printk(KERN_WARNING
			       "hda-intel: Invalid position buffer, "
			       "using LPIB read method instead.\n");
			chip->position_fix[stream] = POS_FIX_LPIB;
			pos = azx_get_position(chip, azx_dev);
		} else
			chip->position_fix[stream] = POS_FIX_POSBUF;
	}

	if (WARN_ONCE(!azx_dev->period_bytes,
		      "hda-intel: zero azx_dev->period_bytes"))
		return -1;	/* this shouldn't happen! */
/* ls2h */
#if 0
	if (wallclk < (azx_dev->period_wallclk * 5) / 4 &&
	    pos % azx_dev->period_bytes > azx_dev->period_bytes / 2)
		/* NG - it's below the first next period boundary */
		return bdl_pos_adj[chip->dev_index] ? 0 : -1;
	azx_dev->start_wallclk += wallclk;
#endif

	/* ls2h: check bogus (too early) interrupt. */

	return 1;		/* OK, it's fine */
}

/* clear irq_pending flags and assure no on-going workq */
static void azx_clear_irq_pending(struct azx *chip)
{
	int i;

	spin_lock_irq(&chip->reg_lock);
	for (i = 0; i < chip->num_streams; i++)
		chip->azx_dev[i].irq_pending = 0;
	spin_unlock_irq(&chip->reg_lock);
}

static struct snd_pcm_ops azx_pcm_ops = {
	.open = azx_pcm_open,
	.close = azx_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = azx_pcm_hw_params,
	.hw_free = azx_pcm_hw_free,
	.prepare = azx_pcm_prepare,
	.trigger = azx_pcm_trigger,
	.pointer = azx_pcm_pointer,
	.page = snd_pcm_sgbuf_ops_page,
};

static void azx_pcm_free(struct snd_pcm *pcm)
{
	struct azx_pcm *apcm = pcm->private_data;
	if (apcm) {
		apcm->chip->pcm[pcm->device] = NULL;
		kfree(apcm);
	}
}

static int
azx_attach_pcm_stream(struct hda_bus *bus, struct hda_codec *codec,
		      struct hda_pcm *cpcm)
{
	struct azx *chip = bus->private_data;
	struct snd_pcm *pcm;
	struct azx_pcm *apcm;
	int pcm_dev = cpcm->device;
	int s, err;

	if (pcm_dev >= HDA_MAX_PCMS) {
		snd_printk(KERN_ERR SFX "Invalid PCM device number %d\n",
			   pcm_dev);
		return -EINVAL;
	}
	if (chip->pcm[pcm_dev]) {
		snd_printk(KERN_ERR SFX "PCM %d already exists\n", pcm_dev);
		return -EBUSY;
	}
	err = snd_pcm_new(chip->card, cpcm->name, pcm_dev,
			  cpcm->stream[SNDRV_PCM_STREAM_PLAYBACK].substreams,
			  cpcm->stream[SNDRV_PCM_STREAM_CAPTURE].substreams,
			  &pcm);
	if (err < 0)
		return err;
	strlcpy(pcm->name, cpcm->name, sizeof(pcm->name));
	apcm = kzalloc(sizeof(*apcm), GFP_KERNEL);
	if (apcm == NULL)
		return -ENOMEM;
	apcm->chip = chip;
	apcm->codec = codec;
	pcm->private_data = apcm;
	pcm->private_free = azx_pcm_free;
	if (cpcm->pcm_type == HDA_PCM_TYPE_MODEM)
		pcm->dev_class = SNDRV_PCM_CLASS_MODEM;
	chip->pcm[pcm_dev] = pcm;
	cpcm->pcm = pcm;
	for (s = 0; s < 2; s++) {
		apcm->hinfo[s] = &cpcm->stream[s];
		if (cpcm->stream[s].substreams)
			snd_pcm_set_ops(pcm, s, &azx_pcm_ops);
	}
	/* buffer pre-allocation */
	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV_SG,
					      snd_dma_platform_data(chip->dev),
					      1024 * 64, 32 * 1024 * 1024);
	return 0;
}

/*
 * mixer creation - all stuff is implemented in hda module
 */
static int azx_mixer_create(struct azx *chip)
{
	return ls2h_hda_build_controls(chip->bus);
}

/*
 * initialize SD streams
 */
static int azx_init_stream(struct azx *chip)
{
	int i;

	/* initialize each stream (aka device)
	 * assign the starting bdl address to each stream (device)
	 * and initialize
	 */
	for (i = 0; i < chip->num_streams; i++) {
		struct azx_dev *azx_dev = &chip->azx_dev[i];
		azx_dev->posbuf = (u32 __iomem *) (chip->posbuf.area + i * 8);
		/* offset: SDI0=0x80, SDI1=0xa0, ... SDO3=0x160 */
		azx_dev->sd_addr = chip->remap_addr + (0x20 * i + 0x80);
		/* int mask: SDI0=0x01, SDI1=0x02, ... SDO3=0x80 */
		azx_dev->sd_int_sta_mask = 1 << i;
		/* stream tag: must be non-zero and unique */
		azx_dev->index = i;
		azx_dev->stream_tag = i + 1;
	}

	return 0;
}

static int azx_acquire_irq(struct azx *chip, int do_disconnect)
{
	/* ls2h: get platform device irq. */
	int irq;
	irq = platform_get_irq(chip->dev, 0);

	if (request_irq(irq, azx_interrupt,
			chip->msi ? 0 : IRQF_SHARED, "ls2h-audio", chip)) {
		printk(KERN_ERR SFX "unable to grab IRQ %d, "
		       "disabling device\n", irq);
		if (do_disconnect)
			snd_card_disconnect(chip->card);
		return -1;
	}
	chip->irq = irq;
	return 0;
}

static void azx_stop_chip(struct azx *chip)
{
	if (!chip->initialized)
		return;

	/* disable interrupts */
	azx_int_disable(chip);
	azx_int_clear(chip);

	/* disable CORB/RIRB */
	azx_free_cmd_io(chip);

	/* disable position buffer */
	azx_writel(chip, DPLBASE, 0);
	azx_writel(chip, DPUBASE, 0);

	chip->initialized = 0;
}

#ifdef CONFIG_SND_HDA_POWER_SAVE
/* power-up/down the controller */
static void azx_power_notify(struct hda_bus *bus)
{
	struct azx *chip = bus->private_data;
	struct hda_codec *c;
	int power_on = 0;

	list_for_each_entry(c, &bus->codec_list, list) {
		if (c->power_on) {
			power_on = 1;
			break;
		}
	}
	if (power_on)
		azx_init_chip(chip, 1);
	else if (chip->running && power_save_controller &&
		 !bus->power_keep_link_on)
		azx_stop_chip(chip);
}
#endif /* CONFIG_SND_HDA_POWER_SAVE */

#ifdef CONFIG_PM
/*
 * power management
 */

static int ls2h_hda_codecs_inuse(struct hda_bus *bus)
{
	struct hda_codec *codec;

	list_for_each_entry(codec, &bus->codec_list, list) {
		if (ls2h_hda_codec_needs_resume(codec))
			return 1;
	}
	return 0;
}

static int azx_suspend(struct platform_device *dev, pm_message_t state)
{
	struct snd_card *card = platform_get_drvdata(dev);
	struct azx *chip = card->private_data;
	int i;

	snd_power_change_state(card, SNDRV_CTL_POWER_D3hot);
	azx_clear_irq_pending(chip);
	for (i = 0; i < HDA_MAX_PCMS; i++)
		snd_pcm_suspend_all(chip->pcm[i]);
	if (chip->initialized)
		ls2h_hda_suspend(chip->bus);
	azx_stop_chip(chip);
	if (chip->irq >= 0) {
		free_irq(chip->irq, chip);
		chip->irq = -1;
	}
	return 0;
}

static int azx_resume(struct platform_device *dev)
{
	struct snd_card *card = platform_get_drvdata(dev);
	struct azx *chip = card->private_data;

	chip->msi = 0;
	if (azx_acquire_irq(chip, 1) < 0)
		return -EIO;

	if (ls2h_hda_codecs_inuse(chip->bus))
		azx_init_chip(chip, 1);

	ls2h_hda_resume(chip->bus);
	snd_power_change_state(card, SNDRV_CTL_POWER_D0);
	return 0;
}
#endif /* CONFIG_PM */

/*
 * reboot notifier for hang-up problem at power-down
 */
static int azx_halt(struct notifier_block *nb, unsigned long event, void *buf)
{
	struct azx *chip = container_of(nb, struct azx, reboot_notifier);
	ls2h_hda_bus_reboot_notify(chip->bus);
	azx_stop_chip(chip);
	return NOTIFY_OK;
}

static void azx_notifier_register(struct azx *chip)
{
	chip->reboot_notifier.notifier_call = azx_halt;
	register_reboot_notifier(&chip->reboot_notifier);
}

static void azx_notifier_unregister(struct azx *chip)
{
	if (chip->reboot_notifier.notifier_call)
		unregister_reboot_notifier(&chip->reboot_notifier);
}

/*
 * destructor
 */
static int azx_free(struct azx *chip)
{
	int i;

	azx_notifier_unregister(chip);

	if (chip->initialized) {
		azx_clear_irq_pending(chip);
		for (i = 0; i < chip->num_streams; i++)
			azx_stream_stop(chip, &chip->azx_dev[i]);
		azx_stop_chip(chip);
	}

	if (chip->irq >= 0)
		free_irq(chip->irq, (void *)chip);
	if (chip->remap_addr)
		iounmap(chip->remap_addr);

	if (chip->azx_dev) {
		for (i = 0; i < chip->num_streams; i++)
			if (chip->azx_dev[i].bdl.area)
				snd_dma_free_pages(&chip->azx_dev[i].bdl);
	}
	if (chip->rb.area)
		snd_dma_free_pages(&chip->rb);
	if (chip->posbuf.area)
		snd_dma_free_pages(&chip->posbuf);
	kfree(chip->azx_dev);
	kfree(chip);

	return 0;
}

static int azx_dev_free(struct snd_device *device)
{
	return azx_free(device->device_data);
}

static int check_position_fix(struct azx *chip, int fix)
{
	switch (fix) {
	case POS_FIX_LPIB:
	case POS_FIX_POSBUF:
		return fix;
	}

/* ls2h */
	return POS_FIX_LPIB;
}

static void check_probe_mask(struct azx *chip, int dev)
{
	chip->codec_probe_mask = probe_mask[dev];
}

/*
 * constructor
 */
static int azx_create(struct snd_card *card,
				struct platform_device *devptr,
				int dev, int driver_type, struct azx **rchip)
{
	struct azx *chip;
	int i, err;
	unsigned short gcap;
	static struct snd_device_ops ops = {
		.dev_free = azx_dev_free,
	};

	*rchip = NULL;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		snd_printk(KERN_ERR SFX "cannot allocate chip\n");
		return -ENOMEM;
	}

	spin_lock_init(&chip->reg_lock);

#ifdef CONFIG_LS2H_SB
	spin_lock_init(&mailbox_lock);
#endif
	mutex_init(&chip->open_mutex);
	chip->card = card;
	chip->dev = devptr;
	chip->irq = -1;
	chip->driver_type = driver_type;
	chip->msi = 0;		/* ls2h: disable MSI for non-pci devices */
	chip->dev_index = dev;

	chip->position_fix[0] = chip->position_fix[1] =
	    check_position_fix(chip, position_fix[dev]);
	check_probe_mask(chip, dev);

	chip->single_cmd = single_cmd;

	if (bdl_pos_adj[dev] < 0) {
		switch (chip->driver_type) {
		case AZX_DRIVER_ICH:
			bdl_pos_adj[dev] = 1;
			break;
		default:
			bdl_pos_adj[dev] = 32;
			break;
		}
	}

	/* ls2h: get addr for platform driver. */
	chip->addr = platform_resource_start(chip->dev, 0);
	chip->remap_addr = ioremap_nocache(chip->addr,
					   platform_resource_len(chip->dev, 0));
	if (chip->remap_addr == NULL) {
		snd_printk(KERN_ERR SFX "ioremap error\n");
		err = -ENXIO;
		goto errout;
	}

	if (azx_acquire_irq(chip, 0) < 0) {
		err = -EBUSY;
		goto errout;
	}

	synchronize_irq(chip->irq);

	gcap = azx_readw(chip, GCAP);
	snd_printdd(SFX "chipset global capabilities = 0x%x\n", gcap);

	/* ls2h: set dma mask directly. */
	/* allow 64bit DMA address if supported by H/W */
/*
	if ((gcap & ICH6_GCAP_64OK) && !dma_set_mask(&chip->dev->dev, DMA_BIT_MASK(64)))
		dma_set_coherent_mask(&chip->dev->dev, DMA_BIT_MASK(64));
	else {
		dma_set_mask(&chip->dev->dev, DMA_BIT_MASK(32));
		dma_set_coherent_mask(&chip->dev->dev, DMA_BIT_MASK(32));
	}
*/
	/* read number of streams from GCAP register instead of using
	 * hardcoded value
	 */
	chip->capture_streams = (gcap >> 8) & 0x0f;
	chip->playback_streams = (gcap >> 12) & 0x0f;
	if (!chip->playback_streams && !chip->capture_streams) {
		/* gcap didn't give any info, switching to old method */
		chip->playback_streams = ICH6_NUM_PLAYBACK;
		chip->capture_streams = ICH6_NUM_CAPTURE;
	}

	chip->capture_index_offset = 0;
	chip->playback_index_offset = chip->capture_streams;
	chip->num_streams = chip->playback_streams + chip->capture_streams;
	chip->playback_streams = 1;
	chip->capture_streams = 1;
	chip->azx_dev = kcalloc(chip->num_streams, sizeof(*chip->azx_dev),
				GFP_KERNEL);
	if (!chip->azx_dev) {
		snd_printk(KERN_ERR SFX "cannot malloc azx_dev\n");
		goto errout;
	}

	for (i = 0; i < chip->num_streams; i++) {
		/* allocate memory for the BDL for each stream */
		err = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV,
					  snd_dma_platform_data(chip->dev),
					  BDL_SIZE, &chip->azx_dev[i].bdl);
		if (err < 0) {
			snd_printk(KERN_ERR SFX "cannot allocate BDL\n");
			goto errout;
		}
	}
	/* allocate memory for the position buffer */
	err = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV,
				  snd_dma_platform_data(chip->dev),
				  chip->num_streams * 8, &chip->posbuf);
	if (err < 0) {
		snd_printk(KERN_ERR SFX "cannot allocate posbuf\n");
		goto errout;
	}
	/* allocate CORB/RIRB */
	err = azx_alloc_cmd_io(chip);
	if (err < 0)
		goto errout;

	/* initialize streams */
	azx_init_stream(chip);

	/* initialize chip */
	azx_init_chip(chip, (probe_only[dev] & 2) == 0);

	/* codec detection */
	if (!chip->codec_mask) {
		snd_printk(KERN_ERR SFX "no codecs found!\n");
		err = -ENODEV;
		goto errout;
	}

	err = snd_device_new(card, SNDRV_DEV_LOWLEVEL, chip, &ops);
	if (err < 0) {
		snd_printk(KERN_ERR SFX "Error creating device [card]!\n");
		goto errout;
	}

	strcpy(card->driver, "hda-intel");
	strlcpy(card->shortname, driver_short_names[chip->driver_type],
		sizeof(card->shortname));
	snprintf(card->longname, sizeof(card->longname),
		 "%s at 0x%lx irq %i", card->shortname, chip->addr, chip->irq);

	*rchip = chip;
	return 0;

errout:
	azx_free(chip);
	return err;
}

static void power_down_all_codecs(struct azx *chip)
{
#ifdef CONFIG_SND_HDA_POWER_SAVE
	/* The codecs were powered up in snd_hda_codec_new().
	 * Now all initialization done, so turn them down if possible
	 */
	struct hda_codec *codec;
	list_for_each_entry(codec, &chip->bus->codec_list, list) {
		ls2h_hda_power_down(codec);
	}
#endif
}

static int azx_probe(struct platform_device *devptr)
{
	static int dev;
	struct snd_card *card;
	struct azx *chip;
	int err;
	struct generic_plat_data *hdata =
		(struct generic_plat_data *)devptr->dev.platform_data;

#ifdef CONFIG_LS2H_SB
	volatile u32 *p;
#endif
	chip_version = hdata->chip_ver;
#ifdef CONFIG_LS2H_SB
	if (chip_version == LS2H_VER2) {
		ls2h_mailbox_addr = ioremap_nocache(0x1bd80130, 0x10);

		/* init mailbox with zero */
		p = (volatile u32 __iomem *)ls2h_mailbox_addr;
		/* set stop bit for hda access request to ls2h */
		p[3] = 0x0;

		/* init io function */
		ls_writel = ls3a_writel;
		ls_writew = ls3a_writew;
		ls_writeb = ls3a_writeb;

		ls_readl = ls3a_readl;
		ls_readw = ls3a_readw;
		ls_readb = ls3a_readb;
	} else {
		ls_readl = readl;
		ls_readw = readw;
		ls_readb = readb;

		ls_writel = writel;
		ls_writew = writew;
		ls_writeb = writeb;
	}
#else
		ls_readl = readl;
		ls_readw = readw;
		ls_readb = readb;

		ls_writel = writel;
		ls_writew = writew;
		ls_writeb = writeb;
#endif

	if (dev >= SNDRV_CARDS)
		return -ENODEV;
	if (!enable[dev]) {
		dev++;
		return -ENOENT;
	}

	err = snd_card_create(index[dev], id[dev], THIS_MODULE, 0, &card);
	if (err < 0) {
		snd_printk(KERN_ERR SFX "Error creating card!\n");
		return err;
	}

	/* set this here since it's referred in ls2h_hda_load_patch() */
	snd_card_set_dev(card, &devptr->dev);

	err = azx_create(card, devptr, dev, AZX_DRIVER_ICH, &chip);
	if (err < 0)
		goto out_free;
	card->private_data = chip;

#ifdef CONFIG_LS2H_HDA_INPUT_BEEP
	chip->beep_mode = beep_mode[dev];
#endif

	/* create codec instances */
	err = azx_codec_create(chip, model[dev]);
	if (err < 0)
		goto out_free;

	if ((probe_only[dev] & 1) == 0) {
		err = azx_codec_configure(chip);
		if (err < 0)
			goto out_free;
	}

	/* create PCM streams */
	err = ls2h_hda_build_pcms(chip->bus);
	if (err < 0)
		goto out_free;

	/* create mixer controls */
	err = azx_mixer_create(chip);
	if (err < 0)
		goto out_free;

	err = snd_card_register(card);
	if (err < 0)
		goto out_free;

	platform_set_drvdata(devptr, card);
	chip->running = 1;
	power_down_all_codecs(chip);
	azx_notifier_register(chip);

	dev++;
	return err;
out_free:
	snd_card_free(card);
	return err;
}

static int __exit azx_remove(struct platform_device *devptr)
{
	snd_card_free(platform_get_drvdata(devptr));
	platform_set_drvdata(devptr, NULL);
	return 0;
}

/* platform_driver definition */
static struct platform_driver driver = {
	.probe = azx_probe,
	.remove = __exit_p(azx_remove),
#ifdef CONFIG_PM
	.suspend = azx_suspend,
	.resume = azx_resume,
#endif
	.driver = {
		   .name = "ls2h-audio",
		   .owner = THIS_MODULE,
		   },
};

static int __init alsa_card_azx_init(void)
{
	return platform_driver_register(&driver);
}

static void __exit alsa_card_azx_exit(void)
{
	platform_driver_unregister(&driver);
}

module_init(alsa_card_azx_init)
module_exit(alsa_card_azx_exit)

/*
 * HD Audio
 */
static struct generic_plat_data ls2h_hda_data = {
	.chip_ver = LS2H_VER3,
};

static struct resource ls2k_audio_resources[] = {
	[0] = {
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ls2k_audio_device = {
	.name           = "ls2h-audio",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(ls2k_audio_resources),
	.resource       = ls2k_audio_resources,
	.dev		= {
		.platform_data = &ls2h_hda_data,
	}
};

/* PCI IDs */
static DEFINE_PCI_DEVICE_TABLE(azx_ids) = {
	/* CPT */
	{ PCI_DEVICE(PCI_VENDOR_ID_LOONGSON, PCI_DEVICE_ID_LOONGSON_HDA), },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, azx_ids);

static int ls2k_hda_probe(struct pci_dev *pdev,
		     const struct pci_device_id *pci_id)
{
	int ret;

	
	/* Enable device in PCI config */
	ret = pci_enable_device(pdev);
	if (ret < 0) {
		printk(KERN_ERR "ls2k hda (%s): Cannot enable PCI device\n",
		       pci_name(pdev));
		goto err_out;
	}

	/* request the mem regions */
	ret = pci_request_region(pdev, 0, "ls2k hda io");
	if (ret < 0) {
		printk( KERN_ERR "ls2k hda (%s): cannot request region 0.\n",
			pci_name(pdev));
		goto err_out;
	}

	ls2k_audio_resources[0].start = pci_resource_start (pdev, 0);
	ls2k_audio_resources[0].end = pci_resource_end(pdev, 0);
	ls2k_audio_resources[1].start = pdev->irq;
	ls2k_audio_resources[1].end = pdev->irq;


	platform_device_register(&ls2k_audio_device);


	return 0;
err_out:
	return ret;
}

static void ls2k_hda_remove(struct pci_dev *pdev)
{
	pci_release_region(pdev, 0);
}

/* pci_driver definition */
static struct pci_driver azx_driver = {
	.name = KBUILD_MODNAME,
	.id_table = azx_ids,
	.probe = ls2k_hda_probe,
	.remove = ls2k_hda_remove,
};

module_pci_driver(azx_driver);
