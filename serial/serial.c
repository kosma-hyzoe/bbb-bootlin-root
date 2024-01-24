// SPDX-License-Identifier: GPL-2.0
#define DEBUG

#include "asm-generic/errno-base.h"
#include "asm/vdso/processor.h"
#include "linux/completion.h"
#include "linux/device/driver.h"
#include "linux/dma-direction.h"
#include "linux/err.h"
#include "linux/irqreturn.h"
#include "linux/limits.h"
#include <linux/minmax.h>
#include "linux/moduleparam.h"
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/atomic.h>

#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <uapi/linux/serial_reg.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/dmapool.h>

#include <linux/debugfs.h>

#define SERIAL_RESET_COUNTER	0
#define SERIAL_GET_COUNTER	1

#define SERIAL_BUFFSIZE 16

#define OMAP_UART_SCR_DMAMODE_CTL3 0x7
#define OMAP_UART_SCR_TX_TRIG_GRANU1 BIT(6)


struct serial_dev {
	struct miscdevice miscdev;
	struct platform_device *pdev;
	struct device *dev;
	struct resource *res;
	void __iomem *regs;
	u64 counter;
	wait_queue_head_t wait;
	spinlock_t lock;
	struct dma_chan *txchan;
	int txongoing;
	struct completion txcomplete;
	int use_dma;
	dma_addr_t fifo_dma_addr;
	dma_addr_t dma_addr;
	char rx_buf[SERIAL_BUFFSIZE];
	char tx_buf[SERIAL_BUFFSIZE];

	unsigned int buf_rd; /* read index */
	unsigned int buf_wr; /* write index */
};

static struct serial_dev *file_to_serial(struct file *f)
{
	return container_of(f->private_data, struct serial_dev, miscdev);
}

static u32 reg_read(struct serial_dev *serial, unsigned int reg)
{
	return *(u32 *)(serial->regs + (reg << 2));
}

static void reg_write(struct serial_dev *serial, u32 val, unsigned int reg)
{
	*(u32 *)(serial->regs + (reg << 2)) = val;
}

static int config_baud_rate(struct serial_dev *serial,
		struct platform_device *pdev)
{
	int ret;
	unsigned int baud_divisor, uartclk;
	ret = of_property_read_u32(pdev->dev.of_node, "clock-frequency",
			&uartclk);
	if (ret <  0) {
		dev_err(&pdev->dev,
			"clock-frequency property not found in Device Tree\n");
		return ret;
	}
	baud_divisor = uartclk / 16 / 115200;
	reg_write(serial, 0x07, UART_OMAP_MDR1);
	reg_write(serial, 0x00, UART_LCR);
	reg_write(serial, UART_LCR_DLAB, UART_LCR);
	reg_write(serial, baud_divisor & 0xff, UART_DLL);
	reg_write(serial, (baud_divisor >> 8) & 0xff, UART_DLM);
	reg_write(serial, UART_LCR_WLEN8, UART_LCR);

	return 0;
}

static void serial_write_char(struct serial_dev *serial, u8 val)
{
	unsigned long flags;

	spin_lock_irqsave(&serial->lock, flags);

	dev_dbg(serial->dev, "char written: '%c' (%d)", val, val);
	/* wait until transmit hold register is empty */
	while ((reg_read(serial, UART_LSR) & UART_LSR_THRE) == 0)
		cpu_relax();

	reg_write(serial, val, UART_TX);

	spin_unlock_irqrestore(&serial->lock, flags);
}


ssize_t serial_write_pio(struct file *f, const char __user *buf,
		size_t sz, loff_t *off)
{
	u8 c;
	int i;

	struct serial_dev *serial = file_to_serial(f);

	for (i = 0; i < sz; i++) {
		if (get_user(c, buf + i))
			return -EFAULT;

		serial_write_char(serial, c);
		serial->counter++;
		if (c == '\n')
			serial_write_char(serial, '\r');
	}
	*off += sz;

	return sz;
}

int serial_cleanup_dma(struct serial_dev *serial)
{
	dma_release_channel(serial->txchan);
	dmaengine_terminate_sync(serial->txchan);
	dma_unmap_resource(serial->dev, serial->fifo_dma_addr, 4, DMA_TO_DEVICE,
			   0);

	return 0;
}

int serial_init_dma(struct serial_dev *serial)
{
	int ret;

	struct dma_slave_config txconf = {};


	/* requesting the dma channel */
	serial->txchan = dma_request_chan(serial->dev, "tx");
	if (IS_ERR(serial->txchan)) {
		pr_alert("DMA tx channel request failed (%ld)",
				PTR_ERR(serial->txchan));
		return -ENODEV;
	}

	serial->fifo_dma_addr = dma_map_resource(serial->dev,
			serial->res->start + UART_TX * 4, 4, DMA_TO_DEVICE, 0);
	ret = dma_mapping_error(serial->dev, serial->fifo_dma_addr);
	if (ret) {
		pr_alert("Not enough memory to map DMA Resource");
		serial_cleanup_dma(serial);
		return -ret;
	}

	txconf.direction = DMA_MEM_TO_DEV;
	txconf.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	txconf.dst_addr = serial->fifo_dma_addr;
	ret = dmaengine_slave_config(serial->txchan, &txconf);
	if (ret) {
		pr_alert("failed to config dmaengine slave");
		serial_cleanup_dma(serial);
		return -ret;
	}

	reg_write(serial, OMAP_UART_SCR_DMAMODE_CTL3 | OMAP_UART_SCR_TX_TRIG_GRANU1,
		UART_OMAP_SCR);
	return 0;
}

void async_dma_tx_complete(void *serial)
{
	complete(&((struct serial_dev *)serial)->txcomplete);
}

ssize_t serial_write_dma(struct file *f, const char __user *buf,
		size_t sz, loff_t *off)
{
	int to_copy, copied, ret;
	char first;
	unsigned long flags;
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;
	struct serial_dev *serial = file_to_serial(f);

	pr_alert("serial_write called");
	spin_lock_irqsave(&serial->lock, flags);
	if (serial->txongoing) {
		spin_unlock_irqrestore(&serial->lock, flags);
		return -EBUSY;
	}
	serial->txongoing = 1;
	spin_unlock_irqrestore(&serial->lock, flags);
	/* ========================below is chaos============================ */

	to_copy = min_t(size_t, sz, SERIAL_BUFFSIZE);
	copied = copy_from_user(serial->tx_buf, buf, to_copy);

	/* OMAP 8250 UART quirk: need to write the first byte manually */
	first = serial->tx_buf[0];

	/* remap the buffer */
	serial->dma_addr = dma_map_single(serial->dev, serial->tx_buf, sz,
			     DMA_TO_DEVICE);
	ret = dma_mapping_error(serial->dev, serial->dma_addr);
	if (ret)
		return -ret;
	desc = dmaengine_prep_slave_single(serial->txchan,
			serial->dma_addr + 1, to_copy - 1, DMA_MEM_TO_DEV,
			DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc) {
		pr_alert("failed to allcoate DMA descriptor");
		return -EIO;
	}
	cookie = dmaengine_submit(desc);
	ret = dma_submit_error(cookie);
	if (ret) {
		pr_alert("failed to submit damengine cookie");
		return ret;
	}

	init_completion(&serial->txcomplete);
	dma_async_issue_pending(serial->txchan);
	reg_write(serial, first, UART_TX);

	desc->callback = async_dma_tx_complete;
	desc->callback_param = serial;
	wait_for_completion(&serial->txcomplete);

	dma_unmap_single(serial->dev, serial->dma_addr, SERIAL_BUFFSIZE,
			 DMA_TO_DEVICE);

	/* ==================above is chaos===================================*/
	spin_lock_irqsave(&serial->lock, flags);
	serial->txongoing = 0;
	spin_unlock_irqrestore(&serial->lock, flags);


	return 0;
}


ssize_t serial_read(struct file *f, char __user *buf, size_t sz, loff_t *off)
{
	char c;
	int ret;

	struct serial_dev *serial = file_to_serial(f);


	if (serial->buf_wr == serial->buf_rd) {
		ret = wait_event_interruptible(serial->wait,
				serial->buf_wr != serial->buf_rd);
		if (ret)
			return ret;
	}


	c = serial->rx_buf[serial->buf_rd];
	dev_dbg(serial->dev, "char read: %c", c);
	serial->buf_rd = (serial->buf_rd + 1) % SERIAL_BUFFSIZE;

	if (put_user(c, buf))
		return -EFAULT;

	*off += 1;

	return 1;
}

static long serial_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	unsigned int __user *argp = (unsigned int __user *) arg;
	struct serial_dev *serial = file_to_serial(f);

	switch (cmd) {
		case SERIAL_RESET_COUNTER:
			serial->counter = 0;
			break;
		case SERIAL_GET_COUNTER:
			if (put_user(serial->counter, argp))
				return -EFAULT;
			break;
		default:
			return -ENOTTY;
	}
	return 0;
}


static const struct file_operations serial_fops_dma = {
	.owner = THIS_MODULE,
	.read = serial_read,
	.write = serial_write_dma,
	.unlocked_ioctl = serial_ioctl,
};


static const struct file_operations serial_fops_pio = {
	.owner = THIS_MODULE,
	.read = serial_read,
	.write = serial_write_pio,
	.unlocked_ioctl = serial_ioctl,
};

static irqreturn_t serial_interrupt(int irq, void *dev_id)
{
	int val;
	struct serial_dev *serial = (struct serial_dev *) dev_id;

	spin_lock(&serial->lock);

	val = reg_read(serial, UART_RX);
	serial->rx_buf[serial->buf_wr] = val;
	serial->buf_wr = (serial->buf_wr + 1) % SERIAL_BUFFSIZE;

	spin_unlock(&serial->lock);

	wake_up(&serial->wait);
	return IRQ_HANDLED;
}

static int serial_probe(struct platform_device *pdev)
{
	int irq, ret;

	struct serial_dev *serial;
	struct resource *res;

	struct dentry *parent_dir;
	struct dentry *counter_file;


	/* allocation and pointer madness */
	serial = devm_kzalloc(&pdev->dev, sizeof(*serial), GFP_KERNEL);
	if (!serial)
		return -ENOMEM;
	serial->pdev = pdev;
	platform_set_drvdata(pdev, serial);
	serial->use_dma = 0;
	serial->txongoing = 0;

	serial->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(serial->regs))
		return PTR_ERR(serial->regs);
	serial->dev = &pdev->dev;

	/* power management */
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	/* spinlock init */
	spin_lock_init(&serial->lock);

	/* baud rate config */
	ret = config_baud_rate(serial, pdev);
	if (ret < 0) {
		pr_alert("failed to configure baud rate");
		pm_runtime_disable(&pdev->dev);
		return ret;
	}


	/* soft reset */
	reg_write(serial, UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT, UART_FCR);
	reg_write(serial, 0x00, UART_OMAP_MDR1);

	/* retrival of address, name etc. from the device tree */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_alert("failed to retive resource");
		return -1;
	}
	serial->res = res;


	/* interrupts */
	irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(&pdev->dev, irq, serial_interrupt, 0, "serial",
			serial);
	if (ret < 0) {
		pr_alert("failed to retive resource");
		dev_err(&pdev->dev, "failed to register interrupt handler %d\n",
				ret);
		return ret;
	}
	reg_write(serial, UART_IER_RDI, UART_IER);
	init_waitqueue_head(&serial->wait);

	/* dma */
	ret = serial_init_dma(serial);
	serial->use_dma = !ret;

	/* miscdev pointer madness and registration */
	serial->miscdev.name = devm_kasprintf(&pdev->dev, GFP_ATOMIC,
			"serial-%x", res->start);
	serial->miscdev.fops =
		serial->use_dma ? &serial_fops_dma : &serial_fops_pio;
	serial->miscdev.parent = &pdev->dev;
	serial->miscdev.minor = MISC_DYNAMIC_MINOR;
	misc_register(&serial->miscdev);
	pr_info("%s registered with%s DMA\n", serial->miscdev.name,
		serial->use_dma ? "" : "out");

	parent_dir = debugfs_create_dir(serial->miscdev.name, NULL);
	debugfs_create_u64("counter", S_IRWXU, parent_dir, &serial->counter);

	return 0;
}

static int serial_remove(struct platform_device *pdev)
{
	struct serial_dev *serial = platform_get_drvdata(pdev);

	if (serial->use_dma) {
		serial_cleanup_dma(serial);
	}

	misc_deregister(&serial->miscdev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}


static const struct of_device_id serial_of_match[] = {
	{ .compatible = "bootlin,serial" },
	{ }
};

MODULE_DEVICE_TABLE(of, serial_of_match);

static struct platform_driver serial_driver = {
	.driver = {
		.name = "bootlin-serial",
		/* this signals "the module is in charge of this char device" */
		.owner = THIS_MODULE,
		.of_match_table = serial_of_match,
	},
	.probe = serial_probe,
	.remove = serial_remove,
};
module_platform_driver(serial_driver);
MODULE_LICENSE("GPL");
