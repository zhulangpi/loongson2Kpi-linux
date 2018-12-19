#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/spi/spi.h>


/*define spi register */
#define	SPCR	0x00
#define	SPSR	0x01
#define FIFO	0x02
#define	SPER	0x03
#define	PARA	0x04
#define	SFCS	0x05
#define	TIMI	0x06

extern unsigned long bus_clock;
struct ls2k_spi {
	struct work_struct	work;
	spinlock_t			lock;

	struct	list_head	msg_queue;
	struct	spi_master	*master;
	void	__iomem		*base;
	int cs_active;
	unsigned int hz;
	unsigned char spcr, sper;
	struct workqueue_struct	*wq;
};

static inline int set_cs(struct ls2k_spi *ls2k_spi, struct spi_device  *spi, int val);

static void ls2k_spi_write_reg(struct ls2k_spi *spi, 
				unsigned char reg, unsigned char data)
{
	writeb(data, spi->base +reg);
}

static char ls2k_spi_read_reg(struct ls2k_spi *spi, 
				unsigned char reg)
{
	return readb(spi->base + reg);
}

static int ls2k_spi_update_state(struct ls2k_spi *ls2k_spi,struct spi_device *spi,
				    struct spi_transfer *t)
{
	unsigned int hz;
	unsigned int div, div_tmp;
	unsigned int bit;
	unsigned long clk;

	hz  = t ? t->speed_hz : spi->max_speed_hz;

	if (!hz)
		hz = spi->max_speed_hz;

	if (ls2k_spi->hz != hz) {
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

		ls2k_spi->hz = hz;
		ls2k_spi->spcr = div_tmp & 3;
		ls2k_spi->sper = (div_tmp >> 2) & 3;

	}

	return 0;
}



static int ls2k_spi_setup(struct spi_device *spi)
{
	struct ls2k_spi *ls2k_spi;
	
	ls2k_spi = spi_master_get_devdata(spi->master);
	if (spi->bits_per_word %8)
		return -EINVAL;

	if(spi->chip_select >= spi->master->num_chipselect)
		return -EINVAL;

	if(!ls2k_spi_update_state(ls2k_spi, spi, NULL))
	{
		unsigned char val;
		val = ls2k_spi_read_reg(ls2k_spi, SPCR);
		ls2k_spi_write_reg(ls2k_spi, SPCR, (val & ~3) | ls2k_spi->spcr);
		val = ls2k_spi_read_reg(ls2k_spi, SPER);
		ls2k_spi_write_reg(ls2k_spi, SPER, (val & ~3) | ls2k_spi->sper);
	}

	set_cs(ls2k_spi, spi, 1);

	return 0;
}

static int 
ls2k_spi_write_read_8bit( struct spi_device *spi,
  const u8 **tx_buf, u8 **rx_buf, unsigned int num)
{
	struct ls2k_spi *ls2k_spi;
	ls2k_spi = spi_master_get_devdata(spi->master);

	
	if (tx_buf && *tx_buf){
		ls2k_spi_write_reg(ls2k_spi, FIFO, *((*tx_buf)++));
 		while((ls2k_spi_read_reg(ls2k_spi, SPSR) & 0x1) == 1);
	}else{
		ls2k_spi_write_reg(ls2k_spi, FIFO, 0);
 		while((ls2k_spi_read_reg(ls2k_spi, SPSR) & 0x1) == 1);
	}

	if (rx_buf && *rx_buf) {
		*(*rx_buf)++ = ls2k_spi_read_reg(ls2k_spi, FIFO);
	}else{
		  ls2k_spi_read_reg(ls2k_spi, FIFO);
	}

	return 1;
}


static unsigned int
ls2k_spi_write_read(struct spi_device *spi, struct spi_transfer *xfer)
{
	struct ls2k_spi *ls2k_spi;
	unsigned int count;
	const u8 *tx = xfer->tx_buf;
	u8 *rx = xfer->rx_buf;

	ls2k_spi = spi_master_get_devdata(spi->master);
	count = xfer->len;

	do {
		if (ls2k_spi_write_read_8bit(spi, &tx, &rx, count) < 0)
			goto out;
		count--;
	} while (count);

out:
	return xfer->len - count;

}

static inline int set_cs(struct ls2k_spi *ls2k_spi, struct spi_device  *spi, int val)
{
		ls2k_spi_write_reg(ls2k_spi, SFCS, val?0xff:0xff^(0x10 << spi->chip_select));
		ls2k_spi_read_reg(ls2k_spi, SFCS);
		return 0;
}

static void ls2k_spi_work(struct work_struct *work)
{
	struct ls2k_spi *ls2k_spi = 
		container_of(work, struct ls2k_spi, work);
	int param;

	spin_lock(&ls2k_spi->lock);
	param = ls2k_spi_read_reg(ls2k_spi, PARA);
	ls2k_spi_write_reg(ls2k_spi, PARA, param&~1);
	while (!list_empty(&ls2k_spi->msg_queue)) {

		struct spi_message *m;
		struct spi_device  *spi;
		struct spi_transfer *t = NULL;

		m = container_of(ls2k_spi->msg_queue.next, struct spi_message, queue);

		list_del_init(&m->queue);
		spin_unlock(&ls2k_spi->lock);

		spi = m->spi;

		/*in here set cs*/
		set_cs(ls2k_spi, spi, 0);



		list_for_each_entry(t, &m->transfers, transfer_list) {

			if (t->len)
				m->actual_length +=
					ls2k_spi_write_read(spi, t);
		}

		set_cs(ls2k_spi, spi, 1);
		m->complete(m->context);


		spin_lock(&ls2k_spi->lock);
	}

	ls2k_spi_write_reg(ls2k_spi, PARA, param);
	spin_unlock(&ls2k_spi->lock);
}



static int ls2k_spi_transfer(struct spi_device *spi, struct spi_message *m)
{

	struct ls2k_spi	*ls2k_spi;
	struct spi_transfer *t = NULL;
	
	m->actual_length = 0;
	m->status		 = 0;

	if (list_empty(&m->transfers) || !m->complete)
		return -EINVAL;

	ls2k_spi = spi_master_get_devdata(spi->master);

	list_for_each_entry(t, &m->transfers, transfer_list) {
		
		if (t->tx_buf == NULL && t->rx_buf == NULL && t->len) {
			dev_err(&spi->dev,
				"message rejected : "
				"invalid transfer data buffers\n");
			goto msg_rejected;
		}

	/*other things not check*/

	}

	spin_lock(&ls2k_spi->lock);
	list_add_tail(&m->queue, &ls2k_spi->msg_queue);
	queue_work(ls2k_spi->wq, &ls2k_spi->work);
	spin_unlock(&ls2k_spi->lock);

	return 0;
msg_rejected:

	m->status = -EINVAL;
 	if (m->complete)
		m->complete(m->context);
	return -EINVAL;
}

static int __init ls2k_spi_probe(struct platform_device *pdev)
{
	struct spi_master	*master;
	struct ls2k_spi		*spi;
	struct resource		*res;
	int ret;
	
	master = spi_alloc_master(&pdev->dev, sizeof(struct ls2k_spi));
	
	if (master == NULL) {
		dev_dbg(&pdev->dev, "master allocation failed\n");
		return-ENOMEM;
	}

	if (pdev->id != -1)
		master->bus_num	= pdev->id;

		master->setup = ls2k_spi_setup;
		master->transfer = ls2k_spi_transfer;
		master->num_chipselect = 4;

	dev_set_drvdata(&pdev->dev, master);

	spi = spi_master_get_devdata(master);

	spi->wq	= create_singlethread_workqueue(pdev->name);
	
	spi->master = master;
	

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto free_master;
	}

	spi->base = ioremap(res->start, (res->end - res->start)+1);
	if (spi->base == NULL) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		ret = -ENXIO;
		goto unmap_io;
	}

	ls2k_spi_write_reg(spi, SPCR, 0x51);
	ls2k_spi_write_reg(spi, SPER, 0x04);
	ls2k_spi_write_reg(spi, TIMI, 0x01);
	//ls2k_spi_write_reg(spi, PARA, 0x46);

	INIT_WORK(&spi->work, ls2k_spi_work);

	spin_lock_init(&spi->lock);
	INIT_LIST_HEAD(&spi->msg_queue);

	ret = spi_register_master(master);
	if (ret < 0)
		goto unmap_io;

	return ret;

unmap_io:
	iounmap(spi->base);
free_master:
	kfree(master);
	spi_master_put(master);
	return ret;

}

static struct platform_driver ls2k_spi_driver = {
	.driver	= {
		.name	= "ls2k-spi",
		.owner	= THIS_MODULE,
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
