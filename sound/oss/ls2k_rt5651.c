/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/sound.h>
#include <linux/soundcard.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
//#include <linux/ac97_codec.h>
#include <linux/ls2k_uda1342.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include "ls2k_uda1342.h"
#include <irq.h>
#include <ls2k_int.h>
#include <linux/i2c.h>
///#define	CONFIG_SND_SB2F_TIMER	1
#define readl(addr)             (*(volatile unsigned int *)CKSEG1ADDR(addr))
#define readq(addr)             (*(volatile unsigned long *)CKSEG1ADDR(addr))
#define writel(val, addr)       *(volatile unsigned int *)CKSEG1ADDR(addr) = (val)
#define writeq(val, addr)       *(volatile unsigned long *)CKSEG1ADDR(addr) = (val)

static int irq0;
static int irq1;

void __iomem *order_addr_in0;
void __iomem *order_addr_in1;
void __iomem *dma_cfg_addr0;
void __iomem *dma_cfg_addr1;
void __iomem *int_sta_addr;
void __iomem *int_cfg_addr;
static int codec_reset = 1;

static int uda1342_volume; 
static struct i2c_client * dvo_client[1];

static irqreturn_t ac97_dma_write_intr(int irq, void *private);
static irqreturn_t ac97_dma_read_intr(int irq, void *private);

static int i2c_read_codec(struct i2c_client *client, unsigned char reg, unsigned char *val);
static int i2c_write_codec(struct i2c_client *client, unsigned char reg, unsigned char *val);
static int ls2k_audio_sync(struct file *file);

#define DMA64 0
#if DMA64
struct dma_desc {
	volatile u32 ordered;
	volatile u32 saddr;
	volatile u32 daddr;
	volatile u32 length;
	volatile u32 step_length;
	volatile u32 step_times;
	volatile u32 cmd;
	volatile u32 stats;
	volatile u32 ordered_hi;
	volatile u32 saddr_hi;
	volatile u32 test1;
	volatile u32 test2;
	volatile u32 test3;
	volatile u32 test4;
	volatile u32 test5;
	volatile u32 test6;
};
#else
struct dma_desc {
	volatile u32 ordered;
	volatile u32 saddr;
	volatile u32 daddr;
	volatile u32 length;
	volatile u32 step_length;
	volatile u32 step_times;
	volatile u32 cmd;
	volatile u32 stats;
};
#endif
struct audio_dma_desc {
	struct dma_desc snd;
	struct dma_desc null;
	struct list_head link;
	struct list_head all;
	dma_addr_t snd_dma_handle;
	dma_addr_t snd_dma;
	u32 pos;
	char *snd_buffer;
};

struct audio_stream {
	struct dma_desc *ask_dma;
	struct list_head free_list;
	struct list_head run_list;
	struct list_head done_list;
	struct list_head all_list;
	spinlock_t lock;
	u32 nbfrags;
	u32 fragsize;
	struct semaphore sem;
	wait_queue_head_t frag_wq;
	dma_addr_t ask_dma_handle;

	int num_channels;
	u32 output;
	u32 sample_rate;
	u32 sample_size;
	u32 rate;
	u32 state;
};

static struct audio_stream input_stream = {
	output: 0,
	fragsize:0x10000,
};

static struct audio_stream output_stream = {
	output: 1,
	fragsize:0x10000,
};

#define uda_address     /*0x1fe70000	*/	0x34

#define uda_dev_addr    0x34

#define uda1342_base    0xffffffffbfe0d000//0xbfe60000
#define DMA_BUF 0x00800000

#define IISVERSION  (volatile unsigned int *)(uda1342_base + 0x0)
#define IISCONFIG   (volatile unsigned int *)(uda1342_base + 0x4)
#define IISSTATE  (volatile unsigned int *)(uda1342_base + 0x8)
#define IISRxData	(0x1fe0d000 + 0xc)
#define	IISTxData	(0x1fe0d000 + 0x10)

#define I2C_SINGLE 0 
#define I2C_BLOCK 1
#define I2C_SMB_BLOCK 2
#define I2C_HW_SINGLE 3 //lxy 

#define GS_SOC_I2C_BASE    0xffffffffbfe01000//0xbfe58000           //0x1fe58000
//#define GS_SOC_I2C_BASE    0xbfe70000 //IIC2's base.
#define GS_SOC_I2C_PRER_LO (volatile unsigned char *)(GS_SOC_I2C_BASE + 0x0)
#define GS_SOC_I2C_PRER_HI (volatile unsigned char *)(GS_SOC_I2C_BASE + 0x1)
#define GS_SOC_I2C_CTR     (volatile unsigned char *)(GS_SOC_I2C_BASE + 0x2)
#define GS_SOC_I2C_TXR     (volatile unsigned char *)(GS_SOC_I2C_BASE + 0x3)
#define GS_SOC_I2C_RXR     (volatile unsigned char *)(GS_SOC_I2C_BASE + 0x3)
#define GS_SOC_I2C_CR      (volatile unsigned char *)(GS_SOC_I2C_BASE + 0x4)
#define GS_SOC_I2C_SR      (volatile unsigned char *)(GS_SOC_I2C_BASE + 0x4)

#define CR_START 0x80
#define CR_WRITE 0x10
#define SR_NOACK 0x80
#define CR_STOP  0x40
#define SR_BUSY  0x40 
#define CR_START 0x80
#define SR_TIP   0x2

#define CR_READ  0x20
#define I2C_WACK 0x8

#define IN3P 1
#define IN1P 0
enum {
	STOP = 0,
	RUN = 1
};

/* Boot options
 * 0 = no VRA, 1 = use VRA if codec supports it
 */
static int      vra = 1;
module_param(vra, bool, 0);
MODULE_PARM_DESC(vra, "if 1 use VRA if codec supports it");

static struct ls2k_audio_state {
	void __iomem *base;
	/* soundcore stuff */
	int dev_audio;
	int dev_mixer;

	struct ac97_codec *codec;
	unsigned codec_base_caps; /* AC'97 reg 00h, "Reset Register" */
	unsigned codec_ext_caps;  /* AC'97 reg 28h, "Extended Audio ID" */
	int no_vra;		/* do not use VRA */

	spinlock_t lock;
	struct mutex open_mutex;
	struct mutex mutex;
	fmode_t open_mode;
	wait_queue_head_t open_wait;

	struct audio_stream	 *input_stream;
	struct audio_stream	 *output_stream;

	u32 rd_ref:1;
	u32 wr_ref:1;
	struct semaphore sem;
} ls2k_audio_state;

static void dma_enable_trans(struct audio_stream * s, struct audio_dma_desc *desc)
{
#if DMA64
	u64 val;
#else
	u32 val;
#endif
	int timeout = 20000;
	unsigned long flags;

	val = desc->snd_dma_handle;
	val |= 0x8;
#if DMA64
	val |= 1;
	local_irq_save(flags);
	writeq(val, (s->output ? order_addr_in0 : order_addr_in1));
#else
	local_irq_save(flags);
	writel(val, (s->output ? order_addr_in0 : order_addr_in1));
#endif
	while ((readl(s->output ? order_addr_in0 : order_addr_in1) & 0x8) && (timeout-- > 0)) {
		udelay(5);
	}
	local_irq_restore(flags);
}

void audio_clear_buf(struct audio_stream *s)
{
	struct audio_dma_desc *desc;

	while (!list_empty(&s->all_list)) {
		desc = list_entry(s->all_list.next, struct audio_dma_desc, all);
		list_del(&desc->all);
		list_del(&desc->link);

		if (desc->snd_buffer)
			free_pages((unsigned long)desc->snd_buffer, get_order(s->fragsize));
		dma_free_coherent(NULL, sizeof(struct audio_dma_desc), desc, 0);
	}

	if (s->ask_dma)
		dma_free_coherent(NULL, sizeof(struct audio_dma_desc), s->ask_dma, 0);

	s->ask_dma = NULL;
}

static void inline link_dma_desc(struct audio_stream *s, struct audio_dma_desc *desc)
{
	spin_lock_irq(&s->lock);

	if(!list_empty(&s->run_list)) {
		struct audio_dma_desc *desc0;
		desc0 = list_entry(s->run_list.prev, struct audio_dma_desc, link);
#if DMA64
		desc0->snd.ordered = (desc->snd_dma_handle | DMA_ORDERED_EN) & 0xffffffff;
		desc0->snd.ordered_hi = (desc->snd_dma_handle | DMA_ORDERED_EN) >> 32;
		desc0->null.ordered = (desc->snd_dma_handle | DMA_ORDERED_EN) & 0xffffffff;
		desc0->null.ordered_hi = (desc->snd_dma_handle | DMA_ORDERED_EN) >> 32;
#else
		desc0->snd.ordered = desc->snd_dma_handle | DMA_ORDERED_EN;
		desc0->null.ordered = desc->snd_dma_handle | DMA_ORDERED_EN;
#endif
		list_add_tail(&desc->link, &s->run_list);
		if(s->state == STOP) {
			s->state = RUN;
			dma_enable_trans(s, desc0);
		}
	}
	else {
		list_add_tail(&desc->link,&s->run_list);
		dma_enable_trans(s, desc);
	}

	spin_unlock_irq(&s->lock);
}

static void ls2k_init_dmadesc(struct audio_stream *s, struct audio_dma_desc *desc, u32 count)
{
	struct dma_desc *_desc;
	u32 control;

	control = s->output ? IISTxData: IISRxData;

	desc->snd.daddr = desc->null.daddr = control;

	_desc = &desc->snd;
#if DMA64
	_desc->ordered = ((desc->snd_dma_handle + sizeof(struct dma_desc)) | DMA_ORDERED_EN) & 0xffffffff;
	_desc->ordered_hi = ((desc->snd_dma_handle + sizeof(struct dma_desc)) | DMA_ORDERED_EN) >> 32;
	_desc->saddr = (desc->snd_dma & 0xffffffff);
	_desc->saddr_hi = desc->snd_dma >> 32;
#else
	_desc->ordered = (desc->snd_dma_handle + sizeof(struct dma_desc)) | DMA_ORDERED_EN;
	_desc->saddr = desc->snd_dma;
#endif
	_desc->length = 8;
	_desc->step_length = 0;
	_desc->step_times = count >> 5;
	_desc->cmd = s->output ? 0x00001001 : 0x00000001; //enable the interrupt

	_desc = &desc->null;
#if DMA64
	_desc->ordered = ((desc->snd_dma_handle + sizeof(struct dma_desc)) | DMA_ORDERED_EN) & 0xffffffff;
	_desc->ordered_hi = ((desc->snd_dma_handle + sizeof(struct dma_desc)) | DMA_ORDERED_EN) >> 32;
	_desc->saddr = (desc->snd_dma & 0xffffffff);
	_desc->saddr_hi = desc->snd_dma >> 32;
#else
	_desc->ordered =  (desc->snd_dma_handle + sizeof(struct dma_desc)) | DMA_ORDERED_EN;
	_desc->saddr = desc->snd_dma;
#endif
	_desc->length = 8;
	_desc->step_length = 0;
	_desc->step_times = 1;
	_desc->cmd = s->output ? 0x00001000 : 0x00000000; //disable the interrupt
}

/* setup buffers,dma descs,buffer. */
int ls2k_setup_buf(struct audio_stream * s)
{	
	int i;
	dma_addr_t dma_phyaddr;

	if (s->ask_dma)
		return -EBUSY;

	for (i=0; i <s->nbfrags; i++) {
		struct audio_dma_desc *desc;

		desc = dma_alloc_coherent(NULL, sizeof(struct audio_dma_desc),
					(dma_addr_t *)&dma_phyaddr, GFP_KERNEL);
		if (!desc) {
			printk(KERN_ERR "2.alloc dma desc err.\n");
			goto err;
		}
		memset(desc, 0, sizeof(struct audio_dma_desc));

		desc->snd_dma_handle = dma_phyaddr;
#if DMA64
		desc->null.ordered = ((dma_phyaddr + sizeof(struct dma_desc)) | DMA_ORDERED_EN) & 0xffffffff;
		desc->null.ordered_hi = ((dma_phyaddr + sizeof(struct dma_desc)) | DMA_ORDERED_EN) >> 32;
		desc->snd.ordered = ((dma_phyaddr + sizeof(struct dma_desc)) | DMA_ORDERED_EN) & 0xffffffff;
		desc->snd.ordered_hi = ((dma_phyaddr + sizeof(struct dma_desc)) | DMA_ORDERED_EN) >> 32;
#else
		desc->null.ordered = (dma_phyaddr + sizeof(struct dma_desc)) | DMA_ORDERED_EN;
		desc->snd.ordered = (dma_phyaddr + sizeof(struct dma_desc)) | DMA_ORDERED_EN;
#endif
		list_add_tail(&desc->link,&s->free_list);
		list_add_tail(&desc->all,&s->all_list);

		desc->snd_buffer = dma_alloc_coherent(NULL, s->fragsize,
					(dma_addr_t *)&dma_phyaddr, GFP_KERNEL);
		if (!desc->snd_buffer) {
			printk(KERN_ERR "4.alloc dma buffer err\n");
			goto err;
		}
		desc->snd_dma = dma_phyaddr;
	}

	/* dma desc for ask_valid one per struct audio_stream */
	s->ask_dma = dma_alloc_coherent(NULL, sizeof(struct dma_desc),
			&dma_phyaddr, GFP_KERNEL);
	if(!s->ask_dma) {
		printk(KERN_ERR "3. alloc dma desc err.\n");
		goto err;
	}
	memset(s->ask_dma, 0, sizeof(struct dma_desc));
	s->ask_dma_handle = dma_phyaddr;

	sema_init(&s->sem, 1);

	return 0;

err:
	audio_clear_buf(s);
	printk(KERN_ERR "unable to allocate audio memory\n");
	return -ENOMEM;
}

static irqreturn_t ac97_dma_read_intr(int irq, void *private)
{
	struct audio_stream *s = (struct audio_stream *)private;
	struct audio_dma_desc *desc;
	unsigned long flags;

	if (list_empty(&s->run_list))
		return IRQ_HANDLED;

	local_irq_save(flags);
	writel(s->ask_dma_handle | 0x4, order_addr_in1);
	while (readl(order_addr_in1) & 4) {
	}
	local_irq_restore(flags);

	do {
		desc = list_entry(s->run_list.next, struct audio_dma_desc, link);
		if (s->ask_dma->ordered == desc->snd.ordered)
			break;

//		spin_lock(&s->lock);
		list_del(&desc->link);
		list_add_tail(&desc->link, &s->done_list);
//		spin_unlock(&s->lock);
	} while(!list_empty(&s->run_list));

	if (!list_empty(&s->done_list))
		wake_up(&s->frag_wq);
	return IRQ_HANDLED;
}

static irqreturn_t ac97_dma_write_intr(int irq, void *private)
{
	struct audio_stream *s = (struct audio_stream *)private;
	struct audio_dma_desc *desc;
	unsigned long flags;

	if (list_empty(&s->run_list))
		return IRQ_HANDLED;

	local_irq_save(flags);
	writel(s->ask_dma_handle | 0x4, order_addr_in0);
	while (readl(order_addr_in0) & 4) {
	}
	local_irq_restore(flags);

	do {
		desc = list_entry(s->run_list.next, struct audio_dma_desc, link);
		/*first desc's ordered may be null*/
		if(s->ask_dma->ordered == desc->snd.ordered || s->ask_dma->ordered == 
			((desc->snd_dma_handle + sizeof(struct dma_desc)) | DMA_ORDERED_EN))
			break;
//		spin_lock(&s->lock);
		list_del(&desc->link);
		desc->pos = 0;
		list_add_tail(&desc->link, &s->free_list);
//		spin_unlock(&s->lock);
	} while(!list_empty(&s->run_list));

	if (!list_empty(&s->free_list))
		wake_up(&s->frag_wq);

	return IRQ_HANDLED;
}

static u32 fill_play_buffer(struct audio_stream *s, const char *buf, u32 count)
{
	struct audio_dma_desc *desc;
	u32 copy_bytes;

	desc = list_entry(s->free_list.next, struct audio_dma_desc, link);
	if(s->num_channels == 1)
	{
		int i,j;
		unsigned char c;
		copy_bytes = min((s->fragsize - desc->pos), count*2)/2;
		
		for(i = 0, j = 0;i < copy_bytes;i += 2, j += 4)
		{
			get_user(c,buf+i);
			*(desc->snd_buffer + desc->pos + j) = c;
			*(desc->snd_buffer + desc->pos + j + 2) = c;
			get_user(c,buf+i+1);
			*(desc->snd_buffer + desc->pos + j + 1) = c;
			*(desc->snd_buffer + desc->pos + j +  3) = c;
		}
		desc->pos += j;
	}
	else
	{
		copy_bytes = min((s->fragsize - desc->pos), count);
		copy_from_user((void *)(desc->snd_buffer + desc->pos), buf, copy_bytes);
		desc->pos += copy_bytes;
	}
	

	if (desc->pos == s->fragsize) {
//		spin_lock_irq(&s->lock);
		list_del(&desc->link);
//		spin_unlock_irq(&s->lock);
		ls2k_init_dmadesc(s, desc, s->fragsize);
		link_dma_desc(s, desc);
	}

	return copy_bytes;
}

static int ls2k_audio_write(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	struct ls2k_audio_state *state = (struct ls2k_audio_state *)file->private_data;
	struct audio_stream *s = state->output_stream;
	const char *buffer0 = buffer;
	unsigned int ret = 0;

	if (*ppos != file->f_pos) {
		return -ESPIPE;
	}
	if (!s->ask_dma && ls2k_setup_buf(s)) {
		return -ENOMEM;
	}

	if (file->f_flags & O_NONBLOCK) {
		if (down_trylock(&s->sem))
			return -EAGAIN;
	} else {
		if (down_interruptible(&s->sem))
			return -ERESTARTSYS;
	}

	while (count > 0) {
		if(list_empty(&s->free_list)) {
			if(file->f_flags & O_NONBLOCK)
				return -EAGAIN;

			if(wait_event_interruptible(s->frag_wq, !list_empty(&s->free_list))) {
				up(&s->sem);
				return -ERESTARTSYS;
			}
		}
		/* Fill data , if the ring is not full */
		ret = fill_play_buffer(s, buffer, count);
		count -= ret;
		buffer += ret;
	}

	up(&s->sem);
	return (buffer - buffer0);
}

static int ls2k_copy_to_user(struct audio_stream *s, char *buffer, u32 count)
{
	struct audio_dma_desc *desc;
	int ret = 0;

	while (!list_empty(&s->done_list) && count) {
		u32 left;
		desc = list_entry(s->done_list.next, struct audio_dma_desc, link);
		left = min(s->fragsize - desc->pos,count);
		copy_to_user(buffer, (void *)(desc->snd_buffer + desc->pos), left);
		desc->pos += left;
		count -= left;
		buffer += left;
		ret += left;
		if (desc->pos == s->fragsize) {
			list_del(&desc->link);
			desc->pos = 0;
			list_add_tail(&desc->link, &s->free_list);
		}
	}

	return ret;
}

static int ls2k_audio_read(struct file *file, char *buffer, size_t count, loff_t * ppos)
{
	struct ls2k_audio_state *state = file->private_data;
	struct audio_stream *s = state->input_stream;
	struct audio_dma_desc *desc;
	char *buffer0 = buffer;

	if (*ppos != file->f_pos)
		return -ESPIPE;

	if (!s->ask_dma && ls2k_setup_buf(s))
		return -ENOMEM;

	/*3. sem:get resource .if 0 ,wait */
	if (file->f_flags & O_NONBLOCK) {
		if (down_trylock(&s->sem))
			return -EAGAIN;
	} else {
		if (down_interruptible(&s->sem))
			return -ERESTARTSYS;
	}

	while (count > 0) {
		int ret;

		while (!list_empty(&s->free_list)) {
			desc = list_entry(s->free_list.next, struct audio_dma_desc, link);
//			spin_lock_irq(&s->lock);
			list_del(&desc->link);
//			spin_unlock_irq(&s->lock);
			ls2k_init_dmadesc(s, desc, s->fragsize);
			link_dma_desc(s, desc);
		}

		/* record's buffer is empty */
		while (list_empty(&s->done_list)) {
			if (file->f_flags & O_NONBLOCK) {
				up(&s->sem);
				return -EAGAIN;
			}
			if(wait_event_interruptible(s->frag_wq, !list_empty(&s->done_list))) {
				up(&s->sem);
				return -ERESTARTSYS;
			}
		}

		/* data is ready now , so copy it */
		ret = ls2k_copy_to_user(s, buffer, count);
		count -= ret;
		buffer += ret;
	}

	while (!list_empty(&s->free_list)) {
		desc = list_entry(s->free_list.next, struct audio_dma_desc, link);
		list_del(&desc->link);
		ls2k_init_dmadesc(s, desc, s->fragsize);
		link_dma_desc(s, desc);
	}

	up(&s->sem);
	return (buffer - buffer0);
}

static void config_uda1342(struct audio_stream *as)
{
        unsigned char rat_cddiv;
        unsigned char rat_bitdiv;
        unsigned char dev_addr;
        unsigned short value;

        dev_addr = uda_dev_addr;
#if 0  //sdram is 120M hz
        rat_bitdiv = 0x27;
        rat_cddiv = 0x4;
#elif 0//sdram is 144M Hz
	rat_bitdiv = 0x2f;
        rat_cddiv = 0x5;
#elif 1	//sysclk is 125 Mhz
        rat_bitdiv = (125000000 / (as->sample_rate * as->sample_size * 2)) / 2 - 1;//0x2b;
        rat_cddiv = (125000000 / (as->sample_rate * 256)) / 2 - 1;//0x5;
#endif
//printk("======>>>sample_rate=%d;sample_size=%d; rat_bitdiv = 0x%x; rat_cddiv=0x%x\n", 	\
					as->sample_rate, as->sample_size, rat_bitdiv, rat_cddiv);
	//writel((readl(DMA_CFG_ADDR0) & ~(7 << 4)) | (1 << 6), DMA_CFG_ADDR0);
	//writel((readl(DMA_CFG_ADDR1) & ~(0x3f << 18)) | (1 << 21), DMA_CFG_ADDR1);//write dma0
	//writel(readl(INT_CFG_ADDR) | (0x1f << 12), INT_CFG_ADDR);//enable dma irq

        * IISCONFIG = (as->sample_size << 24) | (as->sample_size << 16) | (rat_bitdiv << 8) | (rat_cddiv << 0) ;
	if (as->output) {
        	* IISSTATE = 0x0d280;
	}else{
		* IISSTATE = 0x0e800;
	}
        value =0x8000;
}
static long ls2k_audio_ioctl(struct file *file, uint cmd, ulong arg)
{
	struct ls2k_audio_state *state = file->private_data;
	struct audio_stream *os = state->output_stream;
	struct audio_stream *is = state->input_stream;
	int rd=0, wr=0, val=0;
	unsigned char reg_val[2] = {0, 0};

        long    var;
        int     ret,reg;

	if (file->f_mode & FMODE_WRITE)
		wr = 1;
	if (file->f_mode & FMODE_READ)
		rd = 1;

	switch (cmd) {
	case SNDCTL_DSP_RESET:
		if (file->f_mode & FMODE_WRITE)
		{
			ls2k_audio_sync(file);
			audio_clear_buf(os);
		}
		if (file->f_mode & FMODE_READ)
		{
			ls2k_audio_sync(file);
			audio_clear_buf(is);
		}
		return 0;
	case SOUND_MIXER_WRITE_VOLUME:	/*  */
  		ret = get_user(var, (long *) arg); 
		if(ret)
			return ret;
//		printk("\n\n====>>>var is %x\n",var);
		for(reg = 0x2; reg < 4; reg++) {
			i2c_read_codec(dvo_client[0], reg, &reg_val[0]);
			reg_val[0] &= 0xc0;
			reg_val[0] |= (var >> 8) & 0x3f;
			reg_val[1] &= 0xc0;
			reg_val[1] |= var & 0x3f;
			i2c_write_codec(dvo_client[0], reg, &reg_val[0]);
		}
		return 0;
	case OSS_GETVERSION:
		return put_user(SOUND_VERSION, (int *) arg);

	case SNDCTL_DSP_SETDUPLEX:
		return 0;

	case SNDCTL_DSP_GETCAPS:
		val = DSP_CAP_REALTIME | DSP_CAP_TRIGGER | DSP_CAP_MMAP;
		if (is && os)
			val |= DSP_CAP_DUPLEX;
		return put_user(val, (int *) arg);

	case SNDCTL_DSP_SYNC:
		return ls2k_audio_sync(file);

	case SNDCTL_DSP_SPEED:
		if (get_user(val, (int *) arg))
			return -EFAULT;

		if (val < 8000)
			val = 8000;
		if (val > 48000)
			val = 48000;

		if (file->f_mode & FMODE_WRITE) {
			os->sample_rate = val;
			config_uda1342(os);
		}else {
			is->sample_rate = val;
			config_uda1342(is);
		}
		if (put_user(val, (int *) arg))
			return -EFAULT;
		return 0;

	case SNDCTL_DSP_STEREO:
		if (get_user(val, (int *)arg)) {
			return -EFAULT;
			break;
		}

		if (wr)
			os->num_channels = val?2:1;
		else 
			is->num_channels = val?2:1;
		put_user(val, (int *) arg);
		return 0;

	case SNDCTL_DSP_CHANNELS:
		if (get_user(val, (int *) arg))
			return -EFAULT;
		if (val != 0) {
			if (file->f_mode & FMODE_READ) {
				if (val < 0 || val > 2)
					return -EINVAL;
				is->num_channels = val;
			}
			if (file->f_mode & FMODE_WRITE) {
				switch (val) {
				case 1:
				case 2:
					break;
				case 3:
				case 5:
					return -EINVAL;
				case 4:
					if (!(state->codec_ext_caps &
					      AC97_EXTID_SDAC))
						return -EINVAL;
					break;
				case 6:
					if ((state->codec_ext_caps &
					     AC97_EXT_DACS) != AC97_EXT_DACS)
						return -EINVAL;
					break;
				default:
					return -EINVAL;
				}


				os->num_channels = val;
			}
		}
		return put_user(val, (int *) arg);
	case SNDCTL_DSP_GETFMTS:	/* Returns a mask */
		return put_user(AFMT_S16_LE | AFMT_U8, (int *) arg);

	case SNDCTL_DSP_SETFMT:	/* Selects ONE fmt */
		if (get_user(val, (int *) arg))
			return -EFAULT;
		if (val != AFMT_QUERY) {
			if (file->f_mode & FMODE_READ) {
				if (val == AFMT_S16_LE) {
					state->input_stream->sample_size = 16;
					config_uda1342(is);
				}else {
					val = AFMT_U8;
					state->input_stream->sample_size = 8;
					config_uda1342(is);
				}
			}
			if (file->f_mode & FMODE_WRITE) {
				if (val == AFMT_S16_LE) {
					state->output_stream->sample_size = 16;
					config_uda1342(os);
				}else {
					val = AFMT_U8;
					state->output_stream->sample_size = 8;
					config_uda1342(os);
				}
			}
		} else {
			if (file->f_mode & FMODE_READ)
				val = (state->input_stream->sample_size == 16) ?
					AFMT_S16_LE : AFMT_U8;
			else
				val = (state->output_stream->sample_size == 16) ?
					AFMT_S16_LE : AFMT_U8;
		}
		return put_user(val, (int *) arg);

	case SNDCTL_DSP_POST:
		return 0;

	case SNDCTL_DSP_SETFRAGMENT:
		if (get_user(val, (int *)arg)) {
			return -EFAULT;
		}

		if (file->f_mode & FMODE_READ) {
			is->fragsize = 1 << (val & 0xFFFF);
			if (is->fragsize < 1024) is->fragsize = 1024;
			is->nbfrags = (val >> 16) & 0xFFFF;
			if (is->nbfrags < 4) is->nbfrags = 4;
		}

		if (file->f_mode & FMODE_WRITE) {
			os->fragsize = 1 << (val & 0xFFFF);
			if (os->fragsize < 1024) os->fragsize = 1024;
			os->nbfrags = (val >> 16) & 0xFFFF;
			if (os->nbfrags < 4) os->nbfrags = 4;
			if (os->num_channels) {
				//os->fragsize >>= 1;
				os->nbfrags >>= 2;
				if (os->nbfrags < 2) os->nbfrags = 2;
			}
		}

		return 0;

	case SNDCTL_DSP_GETBLKSIZE:
		if (file->f_mode & FMODE_WRITE)
			return put_user(os->fragsize, (int *) arg);
		else
			return put_user(is->fragsize, (int *) arg);

	case SNDCTL_DSP_NONBLOCK:
		spin_lock(&file->f_lock);
		file->f_flags |= O_NONBLOCK;
		spin_unlock(&file->f_lock);
		return 0;

	case SOUND_PCM_READ_RATE:
		return put_user((file->f_mode & FMODE_READ) ?
				state->input_stream->sample_rate :
				state->output_stream->sample_rate,
				(int *)arg);

	case SOUND_PCM_READ_CHANNELS:
		if (file->f_mode & FMODE_READ)
			return put_user(state->input_stream->num_channels, (int *)arg);
		else
			return put_user(state->output_stream->num_channels, (int *)arg);

	case SOUND_PCM_READ_BITS:
		if (file->f_mode & FMODE_READ)
			return put_user(state->input_stream->sample_size, (int *)arg);
		else
			return put_user(state->output_stream->sample_size, (int *)arg);

	case SOUND_PCM_WRITE_FILTER:
	case SNDCTL_DSP_SETSYNCRO:
	case SOUND_PCM_READ_FILTER:
		return -EINVAL;
	default:
		return -EINVAL;
	}

	return 0;	//ls2k_ioctl_mixdev(file, cmd, arg);
}

static int ls2k_audio_sync(struct file *file)
{
        struct ls2k_audio_state *state = file->private_data;
        struct audio_stream *is = state->input_stream;
        struct audio_stream *os = state->output_stream;

        if (file->f_mode & FMODE_READ) {
                if (is->state == STOP && !list_empty(&is->run_list)) {
                        struct audio_dma_desc *desc;
                        desc = list_entry(is->run_list.next, struct audio_dma_desc, link);
                        dma_enable_trans(is, desc);
                        is->state = RUN;
                }

                if (!list_empty(&is->run_list))
                        schedule_timeout(CONFIG_HZ*2);

                /* stop write ac97 dma */
                writel(0x10, order_addr_in1);
        }
        if (file->f_mode & FMODE_WRITE) {
                if (os->state == STOP && !list_empty(&os->run_list)) {
                        struct audio_dma_desc *desc;
                        desc = list_entry(os->run_list.next, struct audio_dma_desc, link);
                        dma_enable_trans(os, desc);
                        os->state = RUN;
                }

                if (!list_empty(&os->run_list))
                        schedule_timeout(CONFIG_HZ*2);

                /* stop read ac97 dma */
                writel(0x10, order_addr_in0);
        }

        return 0;
}

static int ls2k_audio_release(struct inode *inode, struct file *file)
{
        struct ls2k_audio_state *state = file->private_data;

        down(&state->sem);

        if (file->f_mode & FMODE_READ) {
                ls2k_audio_sync(file);
                audio_clear_buf(state->input_stream);
                state->rd_ref = 0;
                free_irq(irq1, state->input_stream);
        }

        if (file->f_mode & FMODE_WRITE) {
                ls2k_audio_sync(file);
                audio_clear_buf(state->output_stream);
                state->wr_ref = 0;
                free_irq(irq0, state->output_stream);
        }

        up(&state->sem);
        return 0;
}

static int ls2k_audio_open(struct inode *inode, struct file *file)
{
	struct ls2k_audio_state *state = &ls2k_audio_state;
	struct audio_stream *is = state->input_stream;
	struct audio_stream *os = state->output_stream;
	int minor = MINOR(inode->i_rdev);
	int err;
	u32 x, conf;

	down(&state->sem);

	/* access control */
	err = -ENODEV;
	if ((file->f_mode & FMODE_WRITE) && !os)
		goto out;
	if ((file->f_mode & FMODE_READ) && !is)
		goto out;
	err = -EBUSY;

	if ((file->f_mode & FMODE_WRITE) && state->wr_ref)
		goto out;
	if ((file->f_mode & FMODE_READ) && state->rd_ref)
		goto out;

	file->private_data = state;

	if ((file->f_mode & FMODE_WRITE)) {
		state->wr_ref = 1;
		os->fragsize = AUDIO_FRAGSIZE_DEFAULT;
		os->nbfrags = PLAY_NBFRAGS;
		os->output = 1;
		os->num_channels = 2;
		os->sample_size = 16;
		if ((minor & 0xf) == SND_DEV_DSP16)
			os->sample_size = 16;
		os->sample_rate = 44100;
		init_waitqueue_head(&os->frag_wq);
		os->ask_dma = NULL;
		INIT_LIST_HEAD(&os->free_list);
		INIT_LIST_HEAD(&os->run_list);
		INIT_LIST_HEAD(&os->done_list);
		INIT_LIST_HEAD(&os->all_list);
		spin_lock_init(&os->lock);
		request_irq(irq0, ac97_dma_write_intr, IRQF_SHARED,
				"ac97dma-write", os);
		config_uda1342(os);
	}

	if (file->f_mode & FMODE_READ) {
		state->rd_ref = 1;
		is->fragsize = AUDIO_FRAGSIZE_DEFAULT;
		is->nbfrags = REC_NBFRAGS;
		is->output = 0;
		is->num_channels = 2;
		is->sample_size = 16;
		if ((minor & 0xf) == SND_DEV_DSP16)
			is->sample_size = 16;
		is->sample_rate = 44100;
		init_waitqueue_head(&is->frag_wq);
		is->ask_dma = NULL;
		INIT_LIST_HEAD(&is->free_list);
		INIT_LIST_HEAD(&is->run_list);
		INIT_LIST_HEAD(&is->done_list);
		INIT_LIST_HEAD(&is->all_list);
		spin_lock_init(&is->lock);
		request_irq(irq1, ac97_dma_read_intr, IRQF_SHARED,
				"ac97dma-read", is);
		config_uda1342(is);
	}

	err = 0;

out:
	up(&state->sem);
	printk("err is %d\n",err);
	return err;
}

static long ls2k_ioctl_mixdev(struct file *file, unsigned int cmd, unsigned long arg)
{
        struct ls2k_audio_state *state = (struct ls2k_audio_state *)file->private_data;
        struct ac97_codec *codec = state->codec;

        return /* */0;
}

static int ls2k_open_mixdev(struct inode *inode, struct file *file)
{
        file->private_data = &ls2k_audio_state;
        return 0;
}

static int ls2k_release_mixdev(struct inode *inode, struct file *file)
{
        return 0;
}

static struct file_operations ls2k_dsp_fops = {
	owner:			THIS_MODULE,
	read:			ls2k_audio_read,
	write:			ls2k_audio_write,
	unlocked_ioctl:		ls2k_audio_ioctl,
	open:			ls2k_audio_open,
	release:		ls2k_audio_release,
};

static struct file_operations ls2k_mixer_fops = {
        .owner          = THIS_MODULE,
        .unlocked_ioctl = ls2k_ioctl_mixdev,
        .open           = ls2k_open_mixdev,
        .release        = ls2k_release_mixdev,
};

static DEFINE_SEMAPHORE(ls2k_ac97_mutex);

static int codec_i2c_init();
static int ls2k_audio_probe(struct platform_device *pdev)
{
	struct ls2k_audio_state *state = &ls2k_audio_state;
	struct resource *res;
	int rc;
	unsigned int ls2k_apbdma_cfg;
	int ls2k_apbdma_sel;

	dma_cfg_addr0 = DMA_CFG_ADDR0;
	int_sta_addr = INT_STA_ADDR;
	int_cfg_addr = INT_CFG_ADDR;

	memset(state, 0, sizeof(struct ls2k_audio_state));

	state->input_stream = &input_stream;
	state->output_stream = &output_stream;
	sema_init(&state->sem, 1);

	res = platform_get_resource(pdev, IORESOURCE_DMA , 0);
	if (res == NULL) {
	    dev_err(&pdev->dev, "no DMA 2 resource defined\n");
	    return -ENOENT;
	}
	order_addr_in0 = res->start;

	ls2k_apbdma_cfg = ls2k_readl(LS2K_APBDMA_CFG_REG);
	ls2k_apbdma_cfg &=  ~(LS2K_APBDMA_MASK << LS2K_I2SS_DMA_SHIFT);
	ls2k_apbdma_sel = (res->start - LS2K_DMA0_REG) >> 4;
	ls2k_apbdma_cfg |= (ls2k_apbdma_sel & LS2K_APBDMA_MASK) << LS2K_I2SS_DMA_SHIFT;
	ls2k_writel(ls2k_apbdma_cfg,LS2K_APBDMA_CFG_REG);

	res = platform_get_resource(pdev, IORESOURCE_DMA , 1);
	if (res == NULL) {
	    dev_err(&pdev->dev, "no DMA 3 resource defined\n");
	    return -ENOENT;
	}
	order_addr_in1 = res->start;

	ls2k_apbdma_cfg = ls2k_readl(LS2K_APBDMA_CFG_REG);
	ls2k_apbdma_cfg &=  ~(LS2K_APBDMA_MASK << LS2K_I2SR_DMA_SHIFT);
	ls2k_apbdma_sel = (res->start - LS2K_DMA0_REG) >> 4;
	ls2k_apbdma_cfg |= (ls2k_apbdma_sel & LS2K_APBDMA_MASK) << LS2K_I2SR_DMA_SHIFT;
	ls2k_writel(ls2k_apbdma_cfg,LS2K_APBDMA_CFG_REG);

	irq0 = platform_get_irq(pdev, 0);
	if (irq0 == 0) {
	    dev_err(&pdev->dev, "failed to get interrupt 2 resouce.\n");
	    return -ENOENT;
	}

	irq1 = platform_get_irq(pdev, 1);
	if (irq1 == 0) {
	    dev_err(&pdev->dev, "failed to get interrupt 3 resouce.\n");
	    return -ENOENT;
	}

	res =  platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "cannot find IO resource\n");
		return -ENOENT;
	}

	if (!request_mem_region(res->start, resource_size(res), "ls2k-audio")){
		printk("request mem error\n");
		return -EBUSY;
	}

	state->base = ioremap(res->start, resource_size(res));
	if (!state->base) {
		dev_err(&pdev->dev, "ls2k-audio - failed to map controller\n");
		printk("error iomap\n");
		rc = -ENOMEM;
		goto err_dev0;
	}

	if ((state->dev_audio = register_sound_dsp(&ls2k_dsp_fops, -1)) < 0){
		printk("error!\n");
		goto err_dev1;
	}
	if ((state->dev_mixer = register_sound_mixer(&ls2k_mixer_fops, -1)) < 0){
		printk("error !\n");
		goto err_dev1;		
	}

	if(codec_i2c_init())
		goto err_dev3;		

	printk("register mixer success\n");
	printk("register dsp success\n");
	printk("SOUND_MIXER_WRITE_VOLUME is %x\n",SOUND_MIXER_WRITE_VOLUME);
	return 0;

err_dev3:
	unregister_sound_mixer(state->codec->dev_mixer);
err_dev2:
	unregister_sound_dsp(state->dev_audio);
err_dev1:
	iounmap(state->base);
	release_mem_region(res->start, resource_size(res));
err_dev0:
	///ac97_release_codec(state->codec);

	return rc;
}

/**************codec***************/
static const struct i2c_device_id codec_ids[] = {
	{ "rt5651", 0 },
	{ /* END OF LIST */ }
};

//static struct i2c_client * dvo_client[1];

MODULE_DEVICE_TABLE(i2c, codec_ids);

static int codec_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	dvo_client[0] = client;
//	if(client->adapter->algo->master_xfer)
//		printk("@@@@@@@@@@@ have client->adapter->algo->master_xfer @@@@@@@@@@\n");
	return 0;
}

static int codec_remove(struct i2c_client *client)
{

	i2c_unregister_device(client);
	return 0;
}

static struct i2c_driver eep_driver = {
	.driver = {
		.name = "codec-edid",
		.owner = THIS_MODULE,
	},
	.probe = codec_probe,
	.remove = codec_remove,
	.id_table = codec_ids,
};

static int i2c_read_codec(struct i2c_client *client, unsigned char reg, unsigned char *val)
{
	unsigned char start = reg;	//reg
	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,		//dev_addr
			.flags	= 0,
			.len	= 1,
			.buf	= &start,
		}, {
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 0x2,
			.buf	= val,
		}
	};

	if (i2c_transfer(client->adapter, msgs, 2) == 2) {
		return 0;
	}
	return -1;
}
static int i2c_write_codec(struct i2c_client *client, unsigned char reg, unsigned char *val)
{
	unsigned char msg[3] = {reg, *val, *(val + 1)};	//reg
	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,		//dev_addr
			.flags	= 0,
			.len	= 3,
			.buf	= &msg[0],
		}
	};

	if (i2c_transfer(client->adapter, msgs, 1) == 1) {
		return 0;
	}
	return -1;
}
static int codec_update_bit(struct i2c_client *client, unsigned char reg, unsigned short mask, unsigned short val)
{
	unsigned char reg_val[2] = {0, 0};
	unsigned short tmp;

	if(i2c_read_codec(client, reg, &reg_val[0]))
		return -1;
	tmp = (reg_val[0] << 8) | reg_val[1];
	tmp &= ~mask;
	tmp |= val;
	reg_val[0] = tmp >> 8;
	reg_val[1] = tmp & 0xff;
	if(i2c_write_codec(dvo_client[0], reg, &reg_val[0]))
		return -1;
	return 0;
}

static int set_mic(int in_num)
{
	int err = 0;
	if(in_num & IN3P)
		err = codec_update_bit(dvo_client[0], 0x64, 0x5 << 11, 0x5 << 11);
	if(in_num & IN1P)
		err = codec_update_bit(dvo_client[0], 0x64, 0x11 << 11, 0x11 << 11);

	err = codec_update_bit(dvo_client[0], 0x65, 0x11 << 10, 0x11 << 10);
	err = codec_update_bit(dvo_client[0], 0x61, 0x3 << 1, 0x3 << 1);
	err = codec_update_bit(dvo_client[0], 0x62, 0x1 << 15, 0x1 << 15);

	if(in_num & IN3P) {
		err = codec_update_bit(dvo_client[0], 0x3c, 0xf, 0x7);
		err = codec_update_bit(dvo_client[0], 0x3e, 0xf, 0x7);
//		err = codec_update_bit(dvo_client[0], 0xe, 0xf000, 0x5000);
		err = codec_update_bit(dvo_client[0], 0xe, 0xf000, 0x7000);
	}
	if(in_num & IN1P) {
		err = codec_update_bit(dvo_client[0], 0x3c, 0xf, 0xd);
		err = codec_update_bit(dvo_client[0], 0x3e, 0xf, 0xd);
//		err = codec_update_bit(dvo_client[0], 0xd, 0xf000, 0x5000);
		err = codec_update_bit(dvo_client[0], 0xd, 0xf000, 0x7000);
	}
	err = codec_update_bit(dvo_client[0], 0x27, 0xffff, 0x3820);
	return err;
}

static int codec_i2c_init()
{
unsigned char set_time[]={
/******  reg   bit_hi bit_lo  ****/
        0x63,   0xa8, 0x10,	//bit 4,11,13,15 first power on
	0xfa,   0x00, 0x11,
	0x80,   0x50, 0x00,
	0x81,   0x1f, 0x02,
	0x82,   0x08, 0x00,
	0x64,   0x02, 0x00,
};

static unsigned char set_path[]={
/******  reg   bit_hi bit_lo  ****/
	0x61,   0x98, 0x00,	//power I2S1 & DACL1 & DACR1
        0x2a,   0x12, 0x12,	//bit14/6:Mute Control for DACL/R1 to Stereo DAC Left/Right Mixer
        0x4f,   0x02, 0x78,	//bit0:Mute Control for DACL1 to OUTMIXL
        0x52,   0x02, 0x78,	//bit0:Mute Control for DACR1 to OUTMIXR
/*****  jd  *****/
        0xbc,   0x02, 0x00,
        0xbb,   0x0c, 0x08,
        0x64,   0x02, 0x04,
/****hp out****/
        0x45,   0x40, 0x00,	//bit13:Mute Control for HPOVOL to HPOMIX
        0x02,   0x20, 0x20,	//power on with low vol & 
        0x02,   0x10, 0x10,	//power on with low vol & 
/****line out****/
        0x53,   0xc0, 0x00,
        0x03,   0x20, 0x20,
        0x03,   0x10, 0x10,
        0x05,   0x80, 0x00,
/**** buf ****/
        0x62,   0x08, 0x00,	//bit11: Stereo1 DAC Digital Filter Power
        0x63,   0xf8, 0xfb,
        0x65,   0xc0, 0x00,	//bit14,15	//record:bit10,11
        0x66,   0x3c, 0x00,	//bit10,11:HPOVOLL	//bit12,13:OUTVOLL
        0x6a,   0x00, 0x3d,
        0x6c,   0x3e, 0x00,	//bit9,10:DAC clk2,clk1(all need) //bit12:ADC clk
        0x81,   0x1f, 0x02,	//clock: k, n
        0x82,   0x08, 0x00,	//clock: m
        0x8e,   0x80, 0x19,	//hp amp
        0x8f,   0x31, 0x00,	//depop
};

	int j;
	unsigned char  val[2] = {0,0};

	if (i2c_add_driver(&eep_driver)) {
		pr_err("i2c-%ld No eeprom device register!",eep_driver.id_table->driver_data);
		return -1;
	}

	i2c_write_codec(dvo_client[0], 0x0, &set_time[2]);	//reset: write any val to reg 0x0
		udelay(15000);

	for(j = 0; j < ARRAY_SIZE(set_time) / 3; j++)
		i2c_write_codec(dvo_client[0], set_time[j * 3], &set_time[3 * j + 1]);

	for(j = 0; j < ARRAY_SIZE(set_path) / 3; j++)
		i2c_write_codec(dvo_client[0], set_path[j * 3], &set_path[3 * j + 1]);

	set_mic(IN3P | IN1P);	//we also can read reg to see if hp has mic,then select IN1/3P ?
	return 0;
}
/**************codec***************/

//static int __devexit ls2k_audio_remove(struct platform_device *pdev)
static int ls2k_audio_remove(struct platform_device *pdev)
{
	struct ls2k_audio_state *state = &ls2k_audio_state;
	struct resource *res;

	unregister_sound_dsp(state->dev_audio);
	unregister_sound_mixer(state->codec->dev_mixer);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iounmap(state->base);
	release_mem_region(res->start, resource_size(res));

	///ac97_release_codec(state->codec);

	return 0;
}

#ifdef CONFIG_PM
static int ls2k_audio_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int ls2k_audio_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define ls2k_audio_suspend NULL
#define ls2k_audio_resume NULL
#endif

static struct platform_driver ls2k_audio_driver = {
	.driver = {
		.name	= "ls2k-audio",
		.owner	= THIS_MODULE,
	},
	.probe		= ls2k_audio_probe,
	.remove		= ls2k_audio_remove,
	.suspend	= ls2k_audio_suspend,
	.resume		= ls2k_audio_resume,
};

static int __init ls2k_audio_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&ls2k_audio_driver);
	if (ret)
		printk(KERN_ERR "failed to register ls2k-audio\n");
	return ret;
}

static void __exit ls2k_audio_exit(void)
{
    platform_driver_unregister(&ls2k_audio_driver);
}

module_init(ls2k_audio_init);
module_exit(ls2k_audio_exit);

#ifndef MODULE

static int __init
ls2k_setup(char *options)
{
	char           *this_opt;

	if (!options || !*options)
		return 0;

	while ((this_opt = strsep(&options, ","))) {
		if (!*this_opt)
			continue;
		if (!strncmp(this_opt, "vra", 3)) {
			vra = 1;
		}
	}

	return 1;
}

__setup("ls2k_audio=", ls2k_setup);

#endif /* MODULE */

MODULE_LICENSE("GPL");

