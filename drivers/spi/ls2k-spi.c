#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/module.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <asm/types.h>
#include <asm/io.h>
#include <asm/dma.h>

#define USE_POLL

/*define spi register */
#define	SPCR	0x00
#define	SPSR	0x01
#define SPDR	0x02
#define	SPER	0x03
#define	SPPR	0x04
#define	SPCSR	0x05
#define	SPTR	0x06
#define	SOFTCS	0x05
#define	PARAM	0x04



struct ls2k_spi_devstate {
	unsigned int	hz;
	u8		spcr;
	u8		sper;
};

struct ls2k_spi {
	/* bitbang has to be first */
	struct spi_bitbang	 bitbang;
	struct completion	 done;

	void __iomem		*regs;
	int irq;
	int	len;
	int	count;
	
	/* data buffers */
	const unsigned char	*tx;
	unsigned char		*rx;

	struct resource		*ioarea;
	struct spi_master	*master;
	struct spi_device	*curdev;
	struct device		*dev;
	struct ls2k_spi_info *pdata;
};

static inline struct ls2k_spi *to_hw(struct spi_device *sdev)
{
	return spi_master_get_devdata(sdev->master);
}

static void ls2k_spi_chipsel(struct spi_device *spi, int value)
{
	struct ls2k_spi *hw = to_hw(spi);
	unsigned char cspol = 0;//spi->mode & SPI_CS_HIGH ? 1 : 0;
	unsigned char chip_select = (0x01 << (spi->chip_select + 4));
	int cs = readb(hw->regs + SOFTCS) & ~(0x11<<spi->chip_select);

	switch (value) {
		case BITBANG_CS_INACTIVE:
			if (cspol) {
				writeb((0x1<<spi->chip_select)|cs, hw->regs + SOFTCS);
			} else {
				writeb((0x11<<spi->chip_select)|cs, hw->regs + SOFTCS);
			}
		break;

		case BITBANG_CS_ACTIVE:
			if (cspol) {
				writeb((0x11<<spi->chip_select)|cs, hw->regs + SOFTCS);
			} else {
				writeb((0x1<<spi->chip_select)|cs, hw->regs + SOFTCS);
			}
		break;
	}
}

static int ls2k_spi_update_state(struct spi_device *spi,
				    struct spi_transfer *t)
{
	struct ls2k_spi_devstate *cs = spi->controller_state;
	unsigned int bpw;
	unsigned int hz;
	unsigned int div, div_tmp;
	unsigned int bit;
	unsigned long clk;

	bpw = t ? t->bits_per_word : spi->bits_per_word;
	hz  = t ? t->speed_hz : spi->max_speed_hz;

	if (!bpw)
		bpw = 8;

	if (!hz)
		hz = spi->max_speed_hz;

	if (bpw != 8) {
		dev_err(&spi->dev, "invalid bits-per-word (%d)\n", bpw);
		return -EINVAL;
	}

	if (cs->hz != hz) {
		clk = 100000000;
		div = DIV_ROUND_UP(clk, hz);

		if (div < 2)
			div = 2;

		if (div > 4096)
			div = 4096;

		bit = fls(div) - 1;
		switch(1 << bit) {
			case 16: 
				div_tmp = 2;
				if (div > (1<<bit)) {
					div_tmp++;
				}
				break;
			case 32:
				div_tmp = 3;
				if (div > (1<<bit)) {
					div_tmp += 2;
				}
				break;
			case 8:
				div_tmp = 4;
				if (div > (1<<bit)) {
					div_tmp -= 2;
				}
				break;
			default:
				div_tmp = bit-1;
				if (div > (1<<bit)) {
					div_tmp++;
				}
				break;
		}
		dev_dbg(&spi->dev, "clk = %ld hz = %d div_tmp = %d bit = %d\n", 
		        clk, hz, div_tmp, bit);

		cs->hz = hz;
		cs->spcr = div_tmp & 3;
		cs->sper = (div_tmp >> 2) & 3;
	}

	return 0;
}

static int ls2k_spi_setupxfer(struct spi_device *spi,
				 struct spi_transfer *t)
{
	struct ls2k_spi_devstate *cs = spi->controller_state;
	struct ls2k_spi *hw = to_hw(spi);
	unsigned char val;
	int ret;

	ret = ls2k_spi_update_state(spi, t);
	if (!ret) {
		val = readb(hw->regs + SPCR);
		writeb((val & ~3) | cs->spcr, hw->regs + SPCR);
		val = readb(hw->regs + SPER);
		writeb((val & ~3) | cs->sper, hw->regs + SPER);
	}

	return ret;
}

static int ls2k_spi_setup(struct spi_device *spi)
{
	struct ls2k_spi_devstate *cs = spi->controller_state;
	struct ls2k_spi *hw = to_hw(spi);
	int ret;

	/* allocate settings on the first call */
	if (!cs) {
		cs = kzalloc(sizeof(struct ls2k_spi_devstate), GFP_KERNEL);
		if (!cs) {
			dev_err(&spi->dev, "no memory for controller state\n");
			return -ENOMEM;
		}

		cs->hz = -1;
		spi->controller_state = cs;
	}

	/* initialise the state from the device */
	ret = ls2k_spi_update_state(spi, NULL);
	if (ret)
		return ret;

	spin_lock(&hw->bitbang.lock);
	if (!hw->bitbang.busy) {
		hw->bitbang.chipselect(spi, BITBANG_CS_INACTIVE);
		/* need to ndelay for 0.5 clocktick ? */
	}
	spin_unlock(&hw->bitbang.lock);

	return 0;
}

static void ls2k_spi_cleanup(struct spi_device *spi)
{
	kfree(spi->controller_state);
}

static inline unsigned int hw_txbyte(struct ls2k_spi *hw, int count)
{
	return hw->tx ? hw->tx[count] : 0x00;
}

static int ls2k_spi_txrx(struct spi_device *spi, struct spi_transfer *t)
{
	struct ls2k_spi *hw = to_hw(spi);
		
	hw->tx = t->tx_buf;
	hw->rx = t->rx_buf;
	hw->len = t->len;
	hw->count = 0;

#ifdef USE_POLL
	for(hw->count=0; hw->count < hw->len; hw->count++) {
		writeb(hw_txbyte(hw, hw->count), hw->regs + SPDR);
		while (readb(hw->regs + SPSR) & 0x01) {
			cpu_relax();
		}
		if (hw->rx)
			hw->rx[hw->count] = readb(hw->regs + SPDR);
		else
			readb(hw->regs + SPDR);
	}
#else
//	init_completion(&hw->done);
	/* send the first byte */
	writeb(hw_txbyte(hw, 0), hw->regs + SPDR);
	wait_for_completion(&hw->done);
#endif

	return hw->count;
}

#ifndef USE_POLL
static irqreturn_t ls2k_spi_irq(int irq, void *dev)
{
	struct ls2k_spi *hw = dev;
	unsigned int spsta = readb(hw->regs + SPSR);
	unsigned int count = hw->count;

	writeb(spsta, hw->regs + SPSR);
	
	hw->count++;

	if (hw->rx){
		hw->rx[count] = readb(hw->regs + SPDR);
	} else {/* 由于发送和接收同时进行，即使SPI从设备没有发送有效数据也必须进行读出操作。*/
		readb(hw->regs + SPDR);
	}

	count++;

	if (count < hw->len){
		writeb(hw_txbyte(hw, count), hw->regs + SPDR);
	} else {
		complete(&hw->done);
	}
	
	return IRQ_HANDLED;
}
#endif

static int ls2k_spi_probe(struct platform_device *pdev)
{
	struct ls2k_spi_info *pdata;
	struct ls2k_spi *hw;
	struct spi_master *master;
	struct resource *res;
	int err = 0;
	unsigned char val;

	master = spi_alloc_master(&pdev->dev, sizeof(struct ls2k_spi));
	if (master == NULL) {
		dev_err(&pdev->dev, "No memory for spi_master\n");
		err = -ENOMEM;
		goto err_nomem;
	}

	hw = spi_master_get_devdata(master);
	memset(hw, 0, sizeof(struct ls2k_spi));

	hw->master = spi_master_get(master);
	hw->pdata = pdata = pdev->dev.platform_data;
	hw->dev = &pdev->dev;


	platform_set_drvdata(pdev, hw);
	init_completion(&hw->done);
	
	/* setup the master state. */
	
	/* the spi->mode bits understood by this driver: */
	if (pdev->id != -1)
		master->bus_num	= pdev->id;
	master->mode_bits = SPI_CPOL | SPI_CPHA;

	master->num_chipselect = 4;
	
	/* setup the state for the bitbang driver */
	
	hw->bitbang.master         = hw->master;
	hw->bitbang.setup_transfer = ls2k_spi_setupxfer;
	hw->bitbang.chipselect     = ls2k_spi_chipsel;
	hw->bitbang.txrx_bufs      = ls2k_spi_txrx;
	
	hw->master->setup  = ls2k_spi_setup;
	hw->master->cleanup = ls2k_spi_cleanup;

	dev_dbg(hw->dev, "bitbang at %p\n", &hw->bitbang);

	/* find and map our resources */
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_no_iores;
	}

	hw->ioarea = request_mem_region(res->start, (res->end - res->start)+1,
					pdev->name);

	if (hw->ioarea == NULL) {
		dev_err(&pdev->dev, "Cannot reserve region\n");
		err = -ENXIO;
		goto err_no_iores;
	}

	hw->regs = ioremap(res->start, (res->end - res->start)+1);
	if (hw->regs == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		err = -ENXIO;
		goto err_no_iomap;
	}


	/* program defaults into the registers */
	writeb(0xc0, hw->regs + SPSR);
	val = readb(hw->regs + PARAM);
	val &= 0xfe;
	writeb(val, hw->regs + PARAM);

#ifndef USE_POLL
 	writeb(0xd0, hw->regs + SPCR);
	hw->irq = platform_get_irq(pdev, 0);
	if (hw->irq < 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		err = -ENOENT;
		goto err_no_irq;
	}
	err = request_irq(hw->irq, ls2k_spi_irq, 0, pdev->name, hw);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto err_no_irq;
	}
#else
 	writeb(0x51, hw->regs + SPCR);
#endif
  	writeb(0x04, hw->regs + SPER);
  	writeb(0x01, hw->regs + SPTR);


	/* setup any gpio we can */

	/* register our spi controller */
	err = spi_bitbang_start(&hw->bitbang);
	if (err) {
		dev_err(&pdev->dev, "Failed to register SPI master\n");
		goto err_register;
	}

	return 0;

err_register:

#ifndef USE_POLL
	free_irq(hw->irq, hw);

err_no_irq:
#endif
	iounmap(hw->regs);

err_no_iomap:
	release_resource(hw->ioarea);
	kfree(hw->ioarea);

err_no_iores:
err_no_pdata:
	spi_master_put(hw->master);;

err_nomem:
	return err;
}

static int ls2k_spi_remove(struct platform_device *dev)
{
	struct ls2k_spi *hw = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);

	spi_bitbang_stop(&hw->bitbang);
#ifndef USE_POLL
	free_irq(hw->irq, hw);
#endif
	iounmap(hw->regs);
	
	/* gpio free */
	
	release_resource(hw->ioarea);
	kfree(hw->ioarea);

	spi_master_put(hw->master);
	return 0;
}

#ifdef CONFIG_PM
static int ls2k_spi_suspend(struct device *dev)
{

	return 0;
}

static int ls2k_spi_resume(struct device *dev)
{
	struct ls2k_spi *hw = platform_get_drvdata(to_platform_device(dev));
	unsigned char val;

/* program defaults into the registers */
	writeb(0xc0, hw->regs + SPSR);
	val = readb(hw->regs + PARAM);
	val &= 0xfe;
	writeb(val, hw->regs + PARAM);
 	writeb(0xd0, hw->regs + SPCR);
  	writeb(0x05, hw->regs + SPER);

//	clk_enable(hw->clk);
	return 0;
}

static const struct dev_pm_ops ls2k_spi_pmops = {
	.suspend	= ls2k_spi_suspend,
	.resume		= ls2k_spi_resume,
};

#define LS1B_SPI_PMOPS &ls2k_spi_pmops
#else
#define LS1B_SPI_PMOPS NULL
#endif /* CONFIG_PM */

static struct platform_driver ls2k_spi_driver = {
	.remove    = __exit_p(ls2k_spi_remove),
	.driver    = {
		.name  = "ls2k-spi",
		.owner = THIS_MODULE,
		.pm    = LS1B_SPI_PMOPS,
	},
};

static int __init ls2k_spi_init(void)
{
	return platform_driver_probe(&ls2k_spi_driver, ls2k_spi_probe);
}

static void __exit ls2k_spi_exit(void)
{
	platform_driver_unregister(&ls2k_spi_driver);
}

module_init(ls2k_spi_init);
module_exit(ls2k_spi_exit);

MODULE_DESCRIPTION("loongson 1B SPI Driver");
MODULE_AUTHOR("tanghaifeng <tanghaifeng-gz@loongson.cn");
MODULE_LICENSE("GPL");
