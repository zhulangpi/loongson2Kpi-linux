/*
 *  linux/drivers/mmc/ls2k_mci.h - Loongson ls2k MCI driver
 *
 *  Copyright (C) 2004-2006 maintech GmbH, Thomas Kleffel <tk@maintech.de>
 *
 * Current driver maintained by Ben Dooks and Simtec Electronics
 *  Copyright (C) 2008 Simtec Electronics <ben-linux@fluff.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/mmc/host.h>
#include <linux/platform_device.h>
#include <linux/cpufreq.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/slot-gpio.h>
/*
#include <mach/dma.h>

#include <mach/regs-sdi.h>
#include <mach/regs-gpio.h>

#include <plat/mci.h>
*/
#include "ls2k_mci.h"

#define DRIVER_NAME "ls2k_sdio"


enum dbg_channels {
	dbg_err   = (1 << 0),
	dbg_debug = (1 << 1),
	dbg_info  = (1 << 2),
	dbg_irq   = (1 << 3),
	dbg_sg    = (1 << 4),
	dbg_dma   = (1 << 5),
	dbg_pio   = (1 << 6),
	dbg_fail  = (1 << 7),
	dbg_conf  = (1 << 8),
};

static const int dbgmap_err   = dbg_fail;
static const int dbgmap_info  = dbg_info | dbg_conf;
static const int dbgmap_debug = dbg_err | dbg_debug;

#define dbg(host, channels, args...)		  \
	do {					  \
	if (dbgmap_err & channels) 		  \
		dev_err(&host->pdev->dev, args);  \
	else if (dbgmap_info & channels)	  \
		dev_info(&host->pdev->dev, args); \
	else if (dbgmap_debug & channels)	  \
		dev_dbg(&host->pdev->dev, args);  \
	} while (0)
static void finalize_request(struct ls2k_mci_host *host);
static void ls2k_mci_send_request(struct mmc_host *mmc);
static void ls2k_mci_reset(struct ls2k_mci_host *host);

#ifdef CONFIG_MMC_DEBUG

static void dbg_dumpregs(struct ls2k_mci_host *host, char *prefix)
{
	u32 con, pre, cmdarg, cmdcon, cmdsta, r0, r1, r2, r3, timer, bsize;
	u32 datcon, datcnt, datsta, fsta, imask;

	con 	= readl(host->base + SDICON);
	pre 	= readl(host->base + SDIPRE);
	cmdarg 	= readl(host->base + SDICMDARG);
	cmdcon 	= readl(host->base + SDICMDCON);
	cmdsta 	= readl(host->base + SDICMDSTAT);
	r0 	= readl(host->base + SDIRSP0);
	r1 	= readl(host->base + SDIRSP1);
	r2 	= readl(host->base + SDIRSP2);
	r3 	= readl(host->base + SDIRSP3);
	timer 	= readl(host->base + SDITIMER);
	bsize 	= readl(host->base + SDIBSIZE);
	datcon 	= readl(host->base + SDIDCON);
	datcnt 	= readl(host->base + SDIDCNT);
	datsta 	= readl(host->base + SDIDSTA);
	fsta 	= readl(host->base + SDIFSTA);
	imask   = readl(host->base + host->sdiimsk);

	dbg(host, dbg_debug, "%s  CON:[%08x]  PRE:[%08x]  TMR:[%08x]\n",
				prefix, con, pre, timer);

	dbg(host, dbg_debug, "%s CCON:[%08x] CARG:[%08x] CSTA:[%08x]\n",
				prefix, cmdcon, cmdarg, cmdsta);

	dbg(host, dbg_debug, "%s DCON:[%08x] FSTA:[%08x]"
			       " DSTA:[%08x] DCNT:[%08x]\n",
				prefix, datcon, fsta, datsta, datcnt);

	dbg(host, dbg_debug, "%s   R0:[%08x]   R1:[%08x]"
			       "   R2:[%08x]   R3:[%08x]\n",
				prefix, r0, r1, r2, r3);
}

static void prepare_dbgmsg(struct ls2k_mci_host *host, struct mmc_command *cmd,
			   int stop)
{
	snprintf(host->dbgmsg_cmd, 300,
		 "#%u%s op:%i arg:0x%08x flags:0x08%x retries:%u",
		 host->ccnt, (stop ? " (STOP)" : ""),
		 cmd->opcode, cmd->arg, cmd->flags, cmd->retries);

	if (cmd->data) {
		snprintf(host->dbgmsg_dat, 300,
			 "#%u bsize:%u blocks:%u bytes:%u",
			 host->dcnt, cmd->data->blksz,
			 cmd->data->blocks,
			 cmd->data->blocks * cmd->data->blksz);
	} else {
		host->dbgmsg_dat[0] = '\0';
	}
}

static void dbg_dumpcmd(struct ls2k_mci_host *host, struct mmc_command *cmd,
			int fail)
{
	unsigned int dbglvl = fail ? dbg_fail : dbg_debug;

	if (!cmd)
		return;

	if (cmd->error == 0) {
		dbg(host, dbglvl, "CMD[OK] %s R0:0x%08x\n",
			host->dbgmsg_cmd, cmd->resp[0]);
	} else {
		dbg(host, dbglvl, "CMD[ERR %i] %s Status:%s\n",
			cmd->error, host->dbgmsg_cmd, host->status);
	}

	if (!cmd->data)
		return;

	if (cmd->data->error == 0) {
		dbg(host, dbglvl, "DAT[OK] %s\n", host->dbgmsg_dat);
	} else {
		dbg(host, dbglvl, "DAT[ERR %i] %s DCNT:0x%08x\n",
			cmd->data->error, host->dbgmsg_dat,
			readl(host->base + SDIDCNT));
	}
}
#else
static void dbg_dumpcmd(struct ls2k_mci_host *host,
			struct mmc_command *cmd, int fail) { }

static void prepare_dbgmsg(struct ls2k_mci_host *host, struct mmc_command *cmd,
			   int stop) { }

static void dbg_dumpregs(struct ls2k_mci_host *host, char *prefix) { }

#endif /* CONFIG_MMC_DEBUG */


/**********************************************************************************************************/
/**********************************************************************************************************/
/*********************************  ls2k-sdio-start ***
 * *************************************************************************/
#define CARD_DETECT_IRQ		(64+12) /*gpio12*/

#if 0
/*read card detect gpio value ,wait program*/
int gpio_get_value(int gpio)
{
	return ls2k1_gpio_read_input_pin(gpio);
}
#endif

/**
 * ls2k_mci_enable_irq - enable IRQ, after having disabled it.
 * @host: The device state.
 * @more: True if more IRQs are expected from transfer.
 *
 * Enable the main IRQ if needed after it has been disabled.
 *
 * The IRQ can be one of the following states:
 *	- disabled during IDLE
 *	- disabled whilst processing data
 *	- enabled during transfer
 *	- enabled whilst awaiting SDIO interrupt detection
 */
static void ls2k_mci_enable_irq(struct ls2k_mci_host *host, bool more)
{
	unsigned long flags;
	bool enable = false;

	local_irq_save(flags);

	host->irq_enabled = more;
	host->irq_disabled = false;

	enable = more | host->sdio_irqen;

	if (host->irq_state != enable) {
		host->irq_state = enable;

		if (enable)
			enable_irq(host->irq);
		else
			disable_irq(host->irq);
	}

	local_irq_restore(flags);
}


static inline bool ls2k_mci_host_usedma(struct ls2k_mci_host *host)
{
	return true;
}

/***************************************************************************
 * Description:
 *
 * Version : 1.00
 * Author  : zhanghualiang
 * Language: C
 * Date    : 2013-07-01
 ***************************************************************************/

static void ls2k_mci_disable_irq(struct ls2k_mci_host *host, bool transfer)
{
	unsigned long flags;

	local_irq_save(flags);
	//printk(KERN_DEBUG "%s: transfer %d\n", __func__, transfer);
	host->irq_disabled = transfer;

	if (transfer && host->irq_state) {
		host->irq_state = false;
		disable_irq(host->irq);
	}
	local_irq_restore(flags);
}


/***************************************************************************
 * Description:
 *
 * Version : 1.00
 * Author  : Sunyoung
 * Language: C
 * Date    : 2013-06-20
 ***************************************************************************/

static void ls2k_mci_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct ls2k_mci_host *host = mmc_priv(mmc);
	unsigned long flags;
	u32 con;
	local_irq_save(flags);

	con = readl(host->base + SDICON);
	host->sdio_irqen = enable;

	if (enable == host->sdio_irqen)
		goto same_state;

	if (enable) {
//		*(volatile unsigned int *)(LS1GP_BOARD_INTREG_BASE+0x4) |= 0x1 << 31;  //int0_en bit[31]  //sdio
//		writel(readl(LS1GP_BOARD_INTREG_BASE+0x4)|(0x1 << 31),LS1GP_BOARD_INTREG_BASE+0x4);

		if (!host->irq_state && !host->irq_disabled) {
			host->irq_state = true;
			enable_irq(host->irq);
		}
	} else {

		//writel(readl(0xbfd01044)&(~(0x1 << 31)),0xbfd01044);

		if (!host->irq_enabled && host->irq_state) {
			disable_irq_nosync(host->irq);
			host->irq_state = false;
		}
	}

 same_state:
	local_irq_restore(flags);

//	ls2k_mci_check_sdio_irq(host);
}




static inline void clear_imask(struct ls2k_mci_host *host)
{
	u32 mask = readl(host->base + host->sdiimsk);

	/* preserve the SDIO IRQ mask state */
//	mask &= SDIIMSK_SDIOIRQ;
	writel(mask, host->base + host->sdiimsk);
}



/***************************************************************************
 * Description:  call by pio_tasklet()
 *      
 * Version : 1.00
 * Author  : Sunyoung 
 * Language: C
 * Date    : 2013-11-21
 ***************************************************************************/
static void finalize_request(struct ls2k_mci_host *host)
{
	struct mmc_request *mrq = host->mrq;
	struct mmc_command *cmd;
	int debug_as_failure = 0;
	if (host->complete_what != COMPLETION_FINALIZE)
		return;

	if (!mrq)
		return;
	cmd = host->cmd_is_stop ? mrq->stop : mrq->cmd;

	if (cmd->data && (cmd->error == 0) &&
	    (cmd->data->error == 0)) {
		if (ls2k_mci_host_usedma(host) && (!host->dma_complete)) {
			dbg(host, dbg_dma, "DMA Missing (%d)!\n",
			    host->dma_complete);
			return;
		}
		dma_unmap_sg(mmc_dev(host->mmc), cmd->data->sg, cmd->data->sg_len, cmd->data->flags & MMC_DATA_WRITE ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
	}
	/* Read response from controller. */
	cmd->resp[0] = readl(host->base + SDIRSP0);
	cmd->resp[1] = readl(host->base + SDIRSP1);
	cmd->resp[2] = readl(host->base + SDIRSP2);
	cmd->resp[3] = readl(host->base + SDIRSP3);

	if (cmd->error)
		debug_as_failure = 1;

	if (cmd->data && cmd->data->error)
		debug_as_failure = 1;

	/* Cleanup controller */
	writel(0, host->base + SDICMDARG);
	writel(0, host->base + SDICMDCON);
	clear_imask(host);

	if (cmd->data && cmd->error)
		cmd->data->error = cmd->error;

	if (cmd->data && cmd->data->stop && (!host->cmd_is_stop)) {
		host->cmd_is_stop = 1;
		ls2k_mci_send_request(host->mmc);
		return;
	}

	/* If we have no data transfer we are finished here */
	if (!mrq->data)
		goto request_done;

	/* Calculate the amout of bytes transfer if there was no error */
	if (mrq->data->error == 0) {
		mrq->data->bytes_xfered =
			(mrq->data->blocks * mrq->data->blksz);
	} else {
		mrq->data->bytes_xfered = 0;
	}

	/* If we had an error while transferring data we flush the
	 * DMA channel and the fifo to clear out any garbage. */
	if (mrq->data->error != 0) {
		if (ls2k_mci_host_usedma(host))
			printk(" dbg-yg =============> ls2k_mci dma data error!\r\n");
	}

request_done:
	host->complete_what = COMPLETION_NONE;
	host->mrq = NULL;

//	ls2k_mci_check_sdio_irq(host);
	mmc_request_done(host->mmc, mrq);
}   



/***************************************************************************
 * Description:  sdio irq tasklet
 *          
 * Version : 1.00
 * Author  : Sunyoung 
 * Language: C
 * Date    : 2013-11-21
 ***************************************************************************/


static void pio_tasklet(unsigned long data)
{
	struct ls2k_mci_host *host = (struct ls2k_mci_host *) data;

	ls2k_mci_disable_irq(host, true);

	if (host->complete_what == COMPLETION_FINALIZE) {
		clear_imask(host);
		
		ls2k_mci_enable_irq(host, false);
		finalize_request(host);
	} else
		ls2k_mci_enable_irq(host, true);
}


/***************************************************************************
 * Description:
 *          
 * Version : 1.00
 * Author  : Sunyoung_yg
 * Language: C
 * Date    : 2013-06-19
 ***************************************************************************/
static irqreturn_t ls2k_mci_irq(int irq, void *dev_id)
{
	struct ls2k_mci_host *host = dev_id;
	struct mmc_command *cmd;
	u32 mci_csta, mci_dsta, mci_imsk;
//	spin_lock_irqsave(&host->complete_lock, iflags);

	mci_csta = readl(host->base + SDICMDSTA);
	mci_imsk = readl(host->base + SDIINTMSK);
	mci_dsta = readl(host->base + SDIDSTA);


	if ((host->complete_what == COMPLETION_NONE) ||
	    (host->complete_what == COMPLETION_FINALIZE)) {
		host->status = "nothing to complete";
		clear_imask(host);
		goto irq_out;
	}
	if (!host->mrq) {
		host->status = "no active mrq";
		clear_imask(host);
		goto irq_out;
	}

	cmd = host->cmd_is_stop ? host->mrq->stop : host->mrq->cmd;
/*	
	//若有数据操作，需要等待 数据搬运完成
	while((mci_dsta = readl(host->base + SDIDSTA)) & 0x03);
	mci_imsk = readl(host->base + SDIINTMSK);
*/
	if (!cmd) {
		host->status = "no active cmd";
		clear_imask(host);
		goto irq_out;
	}

	cmd->error = 0;

	if (mci_imsk & SDIIMSK_CMDTIMEOUT) {
		dbg(host, dbg_err, "CMDSTAT: error CMDTIMEOUT\n");
		cmd->error = -ETIMEDOUT;
		host->status = "error: command timeout";
		goto fail_transfer;
	}

	if((mci_imsk & SDIIMSK_CMDSENT)){
		if (host->complete_what == COMPLETION_CMDSENT) {
			host->status = "ok: command sent";
			goto close_transfer;
		}
	}

	if (mci_imsk & SDIIMSK_RESPONSECRC){
		if (cmd->flags & MMC_RSP_CRC) {
			if (host->mrq->cmd->flags & MMC_RSP_136) {
				dbg(host, dbg_irq,
				    "fixup: ignore CRC fail with long rsp\n");
			} else {
				/* note, we used to fail the transfer
				 * here, but it seems that this is just
				 * the hardware getting it wrong.
				 *
				 * cmd->error = -EILSEQ;
				 * host->status = "error: bad command crc";
				 * goto fail_transfer;
				*/
			}
		}
	}

	if ((mci_imsk & SDIIMSK_CMDSENT)) {
		if (host->complete_what == COMPLETION_RSPFIN) {
			host->status = "ok: command response received";
			goto close_transfer;
		}
		if (host->complete_what == COMPLETION_XFERFINISH_RSPFIN)
			host->complete_what = COMPLETION_XFERFINISH;
	}

	if (!cmd->data)
		goto clear_status_bits;

	if (mci_imsk & SDIIMSK_RXCRCFAIL) {
		dbg(host, dbg_err, "bad data crc (outgoing)\n");
		cmd->data->error = -EILSEQ;
		host->status = "error: bad data crc (outgoing)";
		goto fail_transfer;
	}

	if (mci_imsk & SDIIMSK_TXCRCFAIL) {
		dbg(host, dbg_err, "bad data crc (incoming)\n");
		cmd->data->error = -EILSEQ;
		host->status = "error: bad data crc (incoming)";
		goto fail_transfer;
	}

	if (mci_imsk & SDIIMSK_DATATIMEOUT) {
		dbg(host, dbg_err, "data timeout\n");
		cmd->data->error = -ETIMEDOUT;
		host->status = "error: data timeout";
		goto fail_transfer;
	}

	if ((mci_imsk & SDIIMSK_DATAFINISH )){
		if (host->complete_what == COMPLETION_XFERFINISH) {
			host->status = "ok: data transfer completed";
			host->dma_complete = 1;
			goto close_transfer;
		}

		if (host->complete_what == COMPLETION_XFERFINISH_RSPFIN)
			host->complete_what = COMPLETION_RSPFIN;
	}

clear_status_bits:
	goto irq_out;

fail_transfer:
	host->pio_active = XFER_NONE;

close_transfer:
	host->complete_what = COMPLETION_FINALIZE;

	writel(mci_imsk, host->base + SDIINTMSK);  //write "1" clear  imsk ,
	clear_imask(host);
	tasklet_schedule(&host->pio_tasklet);
	goto irq_out;

irq_out:
	writel(mci_imsk, host->base + SDIINTMSK);  //write "1" clear  imsk ,
//	spin_unlock_irqrestore(&host->complete_lock, iflags);
	return IRQ_HANDLED;
}





/*******************************************************/

static void ls2k_mci_set_clk(struct ls2k_mci_host *host, struct mmc_ios *ios)
{
	u32 mci_psc;
	/* Set clock */
	for (mci_psc = 0; mci_psc < 255; mci_psc++) {
		host->real_rate = host->clk_rate / ((mci_psc+1));

		if (host->real_rate <= ios->clock)
			break;
	}
	if (mci_psc > 255)
		mci_psc = 255;

	host->prescaler = mci_psc;
	writel(host->prescaler, host->base + SDIPRE);
	/* If requested clock is 0, real_rate will be 0, too */
	if (ios->clock == 0)
		host->real_rate = 0;
}



static void ls2k_mci_reset(struct ls2k_mci_host *host)
{
	u32 con = readl(host->base + SDICON);

	con |= SDICON_SDRESET;
	writel(con, host->base + SDICON);
	printk(" ls2k_mci host reset()");
}
/************************************************************************/


static void noinline ls2k_mci_send_command(struct ls2k_mci_host *host,
					struct mmc_command *cmd)
{
	u32 ccon;
/*
 *	u32 imsk;
	imsk  = LS1GP_SDIIMSK_CRCSTATUS | LS1GP_SDIIMSK_CMDTIMEOUT |
		LS1GP_SDIIMSK_RESPONSEND | LS1GP_SDIIMSK_CMDSENT |
		LS1GP_SDIIMSK_RESPONSECRC;
	enable_imask(host, imsk);
*/

	if (cmd->data){
		host->complete_what = COMPLETION_XFERFINISH_RSPFIN;
	}
	else if (cmd->flags & MMC_RSP_PRESENT)
		host->complete_what = COMPLETION_RSPFIN;
	else
		host->complete_what = COMPLETION_CMDSENT;

	writel(cmd->arg, host->base + SDICMDARG);
	ccon  = cmd->opcode & SDICMDCON_INDEX;
	ccon |= SDICMDCON_SENDERHOST | SDICMDCON_CMDSTART;

	if (cmd->flags & MMC_RSP_PRESENT)
		ccon |= SDICMDCON_WAITRSP;

	if (cmd->flags & MMC_RSP_136)
		ccon |= SDICMDCON_LONGRSP;
	writel(ccon, host->base + SDICMDCON);
}
/*******************************************************************/


static int ls2k_mci_prepare_dma(struct ls2k_mci_host *host, struct mmc_data *data)
{
	int dma_len, i;
	int rw = data->flags & MMC_DATA_WRITE;

	dma_len = dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
			     rw ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
	if (dma_len == 0)
		return -ENOMEM;

	host->dma_complete = 0;
	host->dmatogo = dma_len;
	for (i = 0; i < dma_len; i++) {
		host->sg_cpu[i].length = sg_dma_len(&data->sg[i])  / 4;  //unit is  word
		host->sg_cpu[i].step_length = 0;
		host->sg_cpu[i].step_times = 1;//data->blocks;
		host->sg_cpu[i].saddr = sg_dma_address(&data->sg[i]);
		host->sg_cpu[i].daddr = host->phys_base+0x40;
		if (data->flags & MMC_DATA_READ) {						  //  dma read
		    
			host->sg_cpu[i].cmd = 0x1<<0;   //bit12:0(default) & disable dma int 
		} else {												  //  dma write
			host->sg_cpu[i].cmd = ((0x1<<12) | (0x1<<0));   //ddr > dev  & disable dma int 
		}

		host->sg_cpu[i].order_addr = host->sg_dma+(i+1)*sizeof(struct ls2k_dma_desc);
		host->sg_cpu[i].order_addr |= 0x1<<0;   //next desc in effect  
	}
		host->sg_cpu[dma_len-1].order_addr &= ~(0x1<<0);  // last order no effect 

		*(volatile unsigned int *) host->dma_order_reg = (host->sg_dma | 0x8 /*| 0x2*/);    //dma start  use dma1
	return 0;
}



static int ls2k_mci_setup_data(struct ls2k_mci_host *host, struct mmc_data *data)
{
	u32 dcon, tmp;

	/* write DCON register */

	if (!data) {
		writel(0, host->base + SDIDCON);
		return 0;
	}

	if ((data->blksz & 3) != 0) {
		/* We cannot deal with unaligned blocks with more than
		 * one block being transferred. */

		if (data->blocks > 1) {
	//		pr_warning("%s: can't do non-word sized block transfers (blksz %d)\n", __func__, data->blksz);
			return -EINVAL;
		}
	}

	dcon  = data->blocks & SDIDCON_BLKNUM_MASK;
	if (host->bus_width == MMC_BUS_WIDTH_4){
		printk(" sdio  bus width : 4 \r\n");
		dcon |= SDIDCON_WIDEBUS;    //bit16
	}

	dcon |= 3 << 14;  //bit15:dma_en bit14:  start
	writel(dcon, host->base + SDIDCON);
	writel(data->blksz, host->base + SDIBSIZE);  //block  size 
	/* write TIMER register */
	writel(0x007FFFFF, host->base + SDITIMER);


	return 0;
}


/*********************************************************************************/
static void ls2k_mci_send_request(struct mmc_host *mmc)
{
	int res;
	struct ls2k_mci_host *host = mmc_priv(mmc);
	struct mmc_request *mrq = host->mrq;
	struct mmc_command *cmd = host->cmd_is_stop ? mrq->stop : mrq->cmd;
	host->ccnt++;
	if (cmd->data) {
		host->dcnt++;

			res = ls2k_mci_setup_data(host, cmd->data);
			if (res) {
				dbg(host, dbg_err, "setup data error %d\n", res);
				cmd->error = res;
				cmd->data->error = res;
				mmc_request_done(mmc, mrq);
				return;
			}
			res = ls2k_mci_prepare_dma(host, cmd->data);
			if (res) {
				dbg(host, dbg_err, "data prepare error %d\n", res);
				cmd->error = res;
				cmd->data->error = res;
				mmc_request_done(mmc, mrq);
				return;
			}
	}

	/* Send command */
	ls2k_mci_send_command(host, cmd);
	/* Enable Interrupt */
	ls2k_mci_enable_irq(host, true);
	/* fix deselect card no irq */
	if(cmd->opcode == MMC_SELECT_CARD && cmd->arg == 0)
	{
		cmd->error = 0;
		mmc_request_done(mmc, mrq);
	}
}


/***************************************************************************
 * Description:
 *          
 * Version : 1.00
 * Author  : Sunyoung 
 * Language: C
 * Date    : 2013-06-20
 ***************************************************************************/

static void ls2k_mci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct ls2k_mci_host *host = mmc_priv(mmc);
	u32 mci_con, val;
	/* Set the power state */
	mci_con = readl(host->base + SDICON);

	switch (ios->power_mode) {
	case MMC_POWER_ON:
	case MMC_POWER_UP:
	/* config mux: sdio mux with spi0,*/
//		val = readl(MISC_CTRL);
//		val &= ~(0x3 << 16);
//		val |= (1<<16);
//		*(volatile unsigned int *)(MISC_CTRL) = val;

		*(volatile unsigned int *)(host->base + SDICON) = 0x100; // reset sdio ctl reg
		mdelay(100);
		*(volatile unsigned int *)(host->base + SDICON) = 0x01; // enable clk
		*(volatile unsigned int *)(host->base + SDIINTEN) = 0x1ff; // enable  int *************
		break;

	case MMC_POWER_OFF:
	default:
			mci_con |= SDICON_SDRESET;
		break;
	}
	ls2k_mci_set_clk(host, ios);
	/* Set CLOCK_ENABLE */
	if (ios->clock)
		mci_con |= SDICON_CLOCKTYPE;   //clk enable
	writel(mci_con, host->base + SDICON);
	if ((ios->power_mode == MMC_POWER_ON) ||
	    (ios->power_mode == MMC_POWER_UP)) {
		dbg(host, dbg_conf, "running at %lukHz (requested: %ukHz).\n",
			host->real_rate/1000, ios->clock/1000);
	} else {
		dbg(host, dbg_conf, "powered down.\n");
	}
	host->bus_width = ios->bus_width;
}
/***************************************************************************
 * Description:
 *
 * Version : 1.00
 * Author  : Sunyoung
 * Language: C
 * Date    : 2013-06-20
 ***************************************************************************/

static int ls2k_mci_card_present(struct mmc_host *mmc)
{
	return 1;
}

/***************************************************************************
 * Description:
 *
 * Version : 1.00
 * Author  : Sunyoung
 * Language: C
 * Date    : 2013-06-20
 ***************************************************************************/
 static void ls2k_mci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct ls2k_mci_host *host = mmc_priv(mmc);
	host->status = "mmc request";
	host->cmd_is_stop = 0;
	host->mrq = mrq;

	if (ls2k_mci_card_present(mmc) == 0) {
		dbg(host, dbg_err, "%s: no medium present\n", __func__);
		host->mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
	} else
		ls2k_mci_send_request(mmc);

}

/***************************************************************************
 * Description:
 *
 * Version : 1.00
 * Author  : Sunyoung
 * Language: C
 * Date    : 2013-06-20
 ***************************************************************************/

static int ls2k_mci_get_ro(struct mmc_host *mmc)
{
	return 0;
}

/***************************************************************************
 * Description:
 *          
 * Version : 1.00
 * Author  : Sunyoung_yg
 * Language: C
 * Date    : 2013-06-20
 ***************************************************************************/
 static struct mmc_host_ops ls2k_mci_ops = {
	.request	= ls2k_mci_request,
	.set_ios	= ls2k_mci_set_ios,
	.get_ro		= ls2k_mci_get_ro,
};
/***************************************************************************
 * Description:
 *
 * Version : 1.00
 * Author  : Sunyoung
 * Language: C
 * Date    : 2013-06-19
 ***************************************************************************/
#if 0
static irqreturn_t ls2k_mci_irq_cd(int irq, void *dev_id)
{
	struct ls2k_mci_host *host = (struct ls2k_mci_host *)dev_id;
	int ret;
	dbg(host, dbg_irq, "card detect\n");
	ret = gpio_get_value(host->pdata->gpio_detect) ? 0 : 1;  /*sd card detect pin*/
	if (ret)
		printk(" sd card is detect\r\n");
	else
		printk("sd card is  removed\r\n");
	mmc_detect_change(host->mmc, msecs_to_jiffies(500));

	return IRQ_HANDLED;
}
#endif
 static irqreturn_t mci_detect_irq(int irq, void *dev_id)
{
	printk(" int mci_detect_irq() \r\n");
	return IRQ_HANDLED;
}

/***************************************************************************
 * Description:
 *
 * Version : 1.00
 * Author  : Sunyoung_yg
 * Language: C
 * Date    : 2013-06-19
 ***************************************************************************/

static struct ls2k_mci_pdata ls2k_mci_def_pdata = {
	/* This is currently here to avoid a number of if (host->pdata)
	 * checks. Any zero fields to ensure reasonable defaults are picked. */
	 .no_wprotect = 0,
	 .no_detect = 0,
};

static struct ls2k_mci_host *hotpug_host;
static int ls2k_mci_hotplug_set(const char *val, struct kernel_param *kp)
{
	mmc_detect_change(hotpug_host->mmc, msecs_to_jiffies(500));
	return 0;
}

module_param_call(hotplug, ls2k_mci_hotplug_set, NULL, NULL, 0664);

/***************************************************************************
 * Description: probe()
 *
 * Version : 1.00
 * Author  : Sunyoung
 * Language: C
 * Date    : 2013-06-19
 ***************************************************************************/

/* #################### probe #######################*/
static int ls2k_mci_probe(struct platform_device *pdev)
{
	struct ls2k_mci_host *host;
	struct mmc_host	*mmc;
	int ret;
	int i;
	unsigned int tmp;
	unsigned long flags;
	struct resource *r;

	mmc = mmc_alloc_host(sizeof(struct ls2k_mci_host), &pdev->dev);//* alloc a    mmc_host + ls2k_mci_host  */ 
	if (!mmc) {
		ret = -ENOMEM;
		goto  probe_free_host;
	}


	host = mmc_priv(mmc);
	host->mmc 	= mmc;
	host->pdev	= pdev;

	host->pdata = pdev->dev.platform_data;
	if (!host->pdata) {
		pdev->dev.platform_data = &ls2k_mci_def_pdata;
		host->pdata = &ls2k_mci_def_pdata;
	}
	
	spin_lock_init(&host->complete_lock);
	tasklet_init(&host->pio_tasklet, pio_tasklet, (unsigned long) host);
	host->sdiimsk	= 0x3c;
	host->sdidata	= 0x40;//ls2k sd card 只能dma 传输 
	host->clk_div	= 0x10; //0x7;

	host->complete_what 	= COMPLETION_NONE;
	host->pio_active 	= XFER_NONE;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "no IO memory resource defined\n");
		ret = -ENODEV;
		goto probe_free_host;
	}


	host->phys_base = r->start;
	host->base = ioremap(r->start, r->end - r->start + 1);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (r == NULL) {
		dev_err(&pdev->dev, "no IO memory resource defined\n");
		ret = -ENODEV;
		goto probe_free_host;
	}
	host->dma_order_reg = ioremap(r->start, r->end - r->start + 1);

	*(volatile unsigned int *)(host->base + SDICON) = 0x100; // reset sdio ctl reg
	mdelay(1);

//**********************************************************

/*config card detect gpio, input mode, trigger mode etc.
 *config sdio int trigger mode.
 */
	/* sdio  detect */
	
//***********************************************************
	*(volatile unsigned int *)(host->base + SDIINTEN) = 0x1ff; // enable all sdio int *************

	host->irq = platform_get_irq(pdev, 0);
	if (host->irq == 0) {
		dev_err(&pdev->dev, "failed to get interrupt resouce.\n");
		ret = -EINVAL;
		goto probe_iounmap;
	}

	hotpug_host = host;
	local_irq_save(flags);
	if (request_irq(host->irq, ls2k_mci_irq, 0, DRIVER_NAME, host)) {
		dev_err(&pdev->dev, "failed to request mci interrupt.\n");
		ret = -ENOENT;
		goto probe_iounmap;
	}

	disable_irq(host->irq);
	host->irq_state = false;
	local_irq_restore(flags);


   // gpio interupt 

	/* depending on the dma state, get a dma channel to use. */

	if (ls2k_mci_host_usedma(host)) {
		host->dma = 0;//dbg-yg ??? 
	}
	host->sg_cpu = dma_alloc_coherent(&pdev->dev, 0x100*sizeof(struct ls2k_dma_desc), &host->sg_dma, GFP_KERNEL);
	memset(host->sg_cpu, 0 , 0x100*sizeof(struct ls2k_dma_desc));

	host->clk_rate = 144000000;  //sdram rate
	mmc->ops 	= &ls2k_mci_ops;
	mmc->ocr_avail	= MMC_VDD_32_33|MMC_VDD_33_34;
#ifdef CONFIG_MMC_LS1GP_HW_SDIO_IRQ
	mmc->caps	= MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ;
#else
	mmc->caps	= MMC_CAP_4_BIT_DATA;
#endif
	mmc->f_min 	= host->clk_rate / (host->clk_div * 256);
	mmc->f_max 	= host->clk_rate / host->clk_div;

	if (host->pdata->ocr_avail)
		mmc->ocr_avail = host->pdata->ocr_avail;

	mmc->max_blk_count	= 4095;
	mmc->max_blk_size	= 4095;
	mmc->max_req_size	= mmc->max_blk_count * mmc->max_blk_size;
	mmc->max_segs		= 1;
	mmc->max_seg_size = mmc->max_req_size;
  	dbg(host, dbg_debug, "mapped mci_base:%p irq:%u irq_cd:%u dma:%u.\n",host->base, host->irq, host->irq_cd, host->dma);

	ret = mmc_add_host(mmc);
	if (ret) {
		dev_err(&pdev->dev, "failed to add mmc host.\n");
		goto free_cpufreq;
	}
	platform_set_drvdata(pdev, mmc);

	return 0;

 free_cpufreq:
//	ls2k_mci_cpufreq_deregister(host);

 free_dmabuf:
//	clk_disable(host->clk);

 clk_free:
//	clk_put(host->clk);

 probe_free_dma:
//	if (ls2k_mci_host_usedma(host))
//		ls2k_dma_free(host->dma, &ls2k_mci_dma_client);

 probe_free_gpio_wp:
	if (!host->pdata->no_wprotect);
		//gpio_free(host->pdata->gpio_wprotect);

 probe_free_gpio_cd:
	if (!host->pdata->no_detect);
//		gpio_free(host->pdata->gpio_detect);

 probe_free_irq_cd:

 probe_free_irq:
	free_irq(host->irq, host);

 probe_iounmap:
	iounmap(host->base);

 probe_free_mem_region:
//	release_mem_region(host->mem->start, resource_size(host->mem));

 probe_free_gpio:
/*	for (i = LS1GP_GPE(5); i <= LS1GP_GPE(10); i++)
		gpio_free(i);
*/
 probe_free_host:
	mmc_free_host(mmc);

 probe_out:
	return ret;
	return 0;
}

static void ls2k_mci_shutdown(struct platform_device *pdev)
{
	struct mmc_host	*mmc = platform_get_drvdata(pdev);
	struct ls2k_mci_host *host = mmc_priv(mmc);


	mmc_remove_host(mmc);
}

static int ls2k_mci_remove(struct platform_device *pdev)
{
	struct mmc_host		*mmc  = platform_get_drvdata(pdev);
	struct ls2k_mci_host	*host = mmc_priv(mmc);
	struct ls2k_mci_pdata *pd = host->pdata;
	int i;

	ls2k_mci_shutdown(pdev);


	free_irq(host->irq, host);

	iounmap(host->base);
	release_mem_region(host->mem->start, (host->mem->end - host->mem->start +1) );

	mmc_free_host(mmc);
	return 0;
}


static struct platform_driver ls2k_mci_driver = {
	.driver	= {
		.name	= "ls2k_sdio",
		.owner	= THIS_MODULE,
	},
	.probe		= ls2k_mci_probe,
	.remove		= ls2k_mci_remove,
	.shutdown	= ls2k_mci_shutdown,
};

static int __init ls2k_mci_init(void)
{
	return platform_driver_register(&ls2k_mci_driver);
}

static void __exit ls2k_mci_exit(void)
{
	platform_driver_unregister(&ls2k_mci_driver);
}

module_init(ls2k_mci_init);
module_exit(ls2k_mci_exit);

MODULE_DESCRIPTION("Loongson ls2k MMC/SD Card Interface driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR(" Loongson embed team");
