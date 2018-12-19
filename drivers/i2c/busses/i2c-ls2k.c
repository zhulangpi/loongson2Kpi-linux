/*
 * Loongson-2K I2C master mode driver
 *
 * Copyright (C) 2013 Loongson Technology Corporation Limited
 *
 * Originally written by JuxinGao
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <ls2k.h>

#define LS2K_I2C_PRER_LO_REG	0x0
#define LS2K_I2C_PRER_HI_REG	0x1
#define LS2K_I2C_CTR_REG    	0x2
#define LS2K_I2C_TXR_REG    	0x3
#define LS2K_I2C_RXR_REG    	0x3
#define LS2K_I2C_CR_REG     	0x4
#define LS2K_I2C_SR_REG     	0x4
#define CONFGMAC		*(volatile unsigned long long*)0x900000001fe10420

#define ls2k_i2c_readb(addr)		readb(dev->base + addr)
#define ls2k_i2c_writeb(val, addr)	writeb(val, dev->base + addr)

#ifdef LS2K_I2C_DEBUG
#define ls2k_i2c_debug(fmt, args...)	printk(KERN_CRIT fmt, ##args)
#else
#define ls2k_i2c_debug(fmt, args...)
#endif

struct ls2k_i2c_dev {
	spinlock_t		lock;
	unsigned int		suspended:1;
	struct device		*dev;
	void __iomem		*base;
	int			irq;
	struct completion	cmd_complete;
	struct resource		*ioarea;
	struct i2c_adapter	adapter;
};

static void ls2k_i2c_stop(struct ls2k_i2c_dev *dev)
{
	int timeout;
	do {
		timeout = 100;
		ls2k_i2c_writeb(CR_STOP, LS2K_I2C_CR_REG);
		while (ls2k_i2c_readb(LS2K_I2C_SR_REG) & SR_BUSY && timeout)
		{
			timeout--;
			if(timeout<10) udelay(1);
		}
	} while (!timeout);
}

static int ls2k_i2c_start(struct ls2k_i2c_dev *dev,
		int dev_addr, int flags)
{
	int retry = 5;
	unsigned char addr = (dev_addr & 0x7f) << 1;
	addr |= (flags & I2C_M_RD)? 1:0;

start:
	ls2k_i2c_writeb(addr, LS2K_I2C_TXR_REG);
	ls2k_i2c_debug("%s <line%d>: i2c device address: 0x%x\n",
			__func__, __LINE__, addr);
	ls2k_i2c_writeb((CR_START | CR_WRITE), LS2K_I2C_CR_REG);
	while (ls2k_i2c_readb(LS2K_I2C_SR_REG) & SR_TIP) ;

	if (ls2k_i2c_readb(LS2K_I2C_SR_REG) & SR_NOACK) {
		ls2k_i2c_stop(dev);
		mdelay(1);
		while (retry--)
			goto start;
		//pr_info("There is no i2c device ack\n");
		return 0;
	}
	return 1;
}

static void ls2k_i2c_init(struct ls2k_i2c_dev *dev)
{
        ls2k_i2c_writeb(0, LS2K_I2C_CTR_REG);
        ls2k_i2c_writeb(0x2c, LS2K_I2C_PRER_LO_REG);
        ls2k_i2c_writeb(0x1, LS2K_I2C_PRER_HI_REG);
        ls2k_i2c_writeb(0x80, LS2K_I2C_CTR_REG);
}

static int ls2k_i2c_read(struct ls2k_i2c_dev *dev,
		unsigned char *buf, int count)
{
	int i;

	for (i = 0; i < count; i++) {
		ls2k_i2c_writeb((i == count - 1)?
				(CR_READ | CR_ACK) : CR_READ,
				LS2K_I2C_CR_REG);
		while (ls2k_i2c_readb(LS2K_I2C_SR_REG) & SR_TIP) ;
		buf[i] = ls2k_i2c_readb(LS2K_I2C_RXR_REG);
		ls2k_i2c_debug("%s <line%d>: read buf[%d] <= %02x\n",
				__func__, __LINE__, i, buf[i]);
        }

        return i;
}

static int ls2k_i2c_write(struct ls2k_i2c_dev *dev,
		unsigned char *buf, int count)
{
        int i;

        for (i = 0; i < count; i++) {
		ls2k_i2c_writeb(buf[i], LS2K_I2C_TXR_REG);
		ls2k_i2c_debug("%s <line%d>: write buf[%d] => %02x\n",
				__func__, __LINE__, i, buf[i]);
		ls2k_i2c_writeb(CR_WRITE, LS2K_I2C_CR_REG);
		while (ls2k_i2c_readb(LS2K_I2C_SR_REG) & SR_TIP) ;

		if (ls2k_i2c_readb(LS2K_I2C_SR_REG) & SR_NOACK) {
			ls2k_i2c_debug("%s <line%d>: device no ack\n",
					__func__, __LINE__);
			ls2k_i2c_stop(dev);
			return 0;
		}
        }

        return i;
}

static int ls2k_i2c_doxfer(struct ls2k_i2c_dev *dev,
		struct i2c_msg *msgs, int num)
{
	struct i2c_msg *m = msgs;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&dev->lock, flags);
	for(i = 0; i < num; i++) {
		if (!ls2k_i2c_start(dev, m->addr, m->flags)) {
			spin_unlock_irqrestore(&dev->lock, flags);
			return -ENODEV;
		}
		if (m->flags & I2C_M_RD)
			ls2k_i2c_read(dev, m->buf, m->len);
		else
			if(ls2k_i2c_write(dev, m->buf, m->len) != m->len)
				break;

		++m;
	}

	ls2k_i2c_stop(dev);
	spin_unlock_irqrestore(&dev->lock, flags);

	return i;
}

static int ls2k_i2c_xfer(struct i2c_adapter *adap,
                        struct i2c_msg *msgs, int num)
{
	int ret;
	int retry;
	struct ls2k_i2c_dev *dev;

	dev = i2c_get_adapdata(adap);
	for (retry = 0; retry < adap->retries; retry++) {
		ret = ls2k_i2c_doxfer(dev, msgs, num);
		if (ret != -EAGAIN)
			return ret;

		udelay(100);
	}

	return -EREMOTEIO;
}

static unsigned int ls2k_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm ls2k_i2c_algo = {
	.master_xfer	= ls2k_i2c_xfer,
	.functionality	= ls2k_i2c_func,
};

static int ls2k_i2c_probe(struct platform_device *pdev)
{
	struct ls2k_i2c_dev	*dev;
	struct i2c_adapter	*adap;
	struct resource		*mem, *irq, *ioarea;
	int r;

	/* NOTE: driver uses the static register mapping */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -ENODEV;
	}
	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return -ENODEV;
	}

	ioarea = request_mem_region(mem->start, resource_size(mem),
			pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "I2C region already claimed\n");
		return -EBUSY;
	}

	dev = kzalloc(sizeof(struct ls2k_i2c_dev), GFP_KERNEL);
	if (!dev) {
		r = -ENOMEM;
		goto err_release_region;
	}

	spin_lock_init(&dev->lock);
	dev->dev = &pdev->dev;
	dev->irq = irq->start;
	dev->base = ioremap(mem->start, resource_size(mem));
	if (!dev->base) {
		r = -ENOMEM;
		goto err_free_mem;
	}

	platform_set_drvdata(pdev, dev);

	ls2k_i2c_init(dev);

	adap = &dev->adapter;
	i2c_set_adapdata(adap, dev);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_HWMON;
	adap->retries = 5;
	strlcpy(adap->name, "LS2K I2C adapter", sizeof(adap->name));
	adap->algo = &ls2k_i2c_algo;
	adap->dev.parent = &pdev->dev;

	/* i2c device drivers may be active on return from add_adapter() */
	adap->nr = pdev->id;
	r = i2c_add_numbered_adapter(adap);
	if (r) {
		dev_err(dev->dev, "failure adding adapter\n");
		goto err_iounmap;
	}

	return 0;

err_iounmap:
	iounmap(dev->base);
err_free_mem:
	platform_set_drvdata(pdev, NULL);
	kfree(dev);
err_release_region:
	release_mem_region(mem->start, resource_size(mem));

	return r;
}

static int ls2k_i2c_remove(struct platform_device *pdev)
{
	struct ls2k_i2c_dev	*dev = platform_get_drvdata(pdev);
	struct resource		*mem;

	platform_set_drvdata(pdev, NULL);
	i2c_del_adapter(&dev->adapter);
	iounmap(dev->base);
	kfree(dev);
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, resource_size(mem));
	return 0;
}

#ifdef CONFIG_PM
static int ls2k_i2c_suspend_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ls2k_i2c_dev *i2c_dev = platform_get_drvdata(pdev);

	i2c_dev->suspended = 1;

	return 0;
}

static int ls2k_i2c_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ls2k_i2c_dev *i2c_dev = platform_get_drvdata(pdev);

	i2c_dev->suspended = 0;
	ls2k_i2c_init(i2c_dev);

	return 0;
}

static const struct dev_pm_ops ls2k_i2c_dev_pm_ops = {
	.suspend_noirq	= ls2k_i2c_suspend_noirq,
	.resume		= ls2k_i2c_resume,
};

#define LS2K_DEV_PM_OPS (&ls2k_i2c_dev_pm_ops)
#else
#define LS2K_DEV_PM_OPS NULL
#endif

static struct platform_driver ls2k_i2c_driver = {
	.probe		= ls2k_i2c_probe,
	.remove		= ls2k_i2c_remove,
	.driver		= {
		.name	= "ls2k-i2c",
		.owner	= THIS_MODULE,
		.pm	= LS2K_DEV_PM_OPS,
	},
};

static int __init ls2k_i2c_init_driver(void)
{
	CONFGMAC |= 0x400;
	return platform_driver_register(&ls2k_i2c_driver);
}
subsys_initcall(ls2k_i2c_init_driver);

static void __exit ls2k_i2c_exit_driver(void)
{
	platform_driver_unregister(&ls2k_i2c_driver);
}

module_exit(ls2k_i2c_exit_driver);

MODULE_AUTHOR("Loongson Technology Corporation Limited");
MODULE_DESCRIPTION("LOONGSON-2K I2C bus adapter");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ls2k-i2c");
