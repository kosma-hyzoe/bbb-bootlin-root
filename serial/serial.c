// SPDX-License-Identifier: GPL-2.0
#include "asm-generic/errno-base.h"
#include "linux/device/driver.h"
#include "linux/irqreturn.h"
#include "linux/limits.h"
#include "linux/moduleparam.h"
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

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
#include <linux/debugfs.h>

#define SERIAL_RESET_COUNTER	0
#define SERIAL_GET_COUNTER	1

struct serial_dev {
	void __iomem *regs;
	struct miscdevice miscdev;
	unsigned int counter;
	struct platform_device *pdev;
};


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
	/* wait until transmit hold register is empty */
	while ((reg_read(serial, UART_LSR) & UART_LSR_THRE) == 0)
		cpu_relax();

	reg_write(serial, val, UART_TX);

	if (val == '\n')
		serial_write_char(serial, '\r');
	else
		serial->counter++;
}


ssize_t serial_write(struct file *file, const char __user *ubuf,
		size_t sz, loff_t *off)
{
	struct miscdevice *miscdev_ptr = file->private_data;
	struct serial_dev *serial = container_of(miscdev_ptr, struct serial_dev,
			miscdev);
	u8 c;
	int i;
	for (i = 0; i < sz; i++) {
		if (get_user(c, ubuf + i))
			return -EFAULT;

		serial_write_char(serial, c);
	}
	*off += sz;

	return sz;
}

ssize_t serial_read(struct file *f, char __user *buf, size_t sz, loff_t *off)
{
	return -1;
}

static long serial_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	unsigned int __user *argp = (unsigned int __user *) arg;
	struct serial_dev *serial = container_of(f->private_data,
			struct serial_dev, miscdev);

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



static const struct file_operations serial_fops = {
	.owner = THIS_MODULE,
	.read = serial_read,
	.write = serial_write,
	.unlocked_ioctl = serial_ioctl,
};

static irqreturn_t serial_interrupt(int irq, void *dev_id)
{
	u32 val;
	struct serial_dev *serial = (struct serial_dev *) dev_id;

	val = reg_read(serial, UART_RX);
	pr_alert("%c\n", val);
	pr_alert("%s called", __func__);
	return IRQ_HANDLED;
}

static int serial_probe(struct platform_device *pdev)
{
	int irq, ret;

	struct serial_dev *serial;
	struct resource *res;

	/* allocation and pointer madness */
	serial = devm_kzalloc(&pdev->dev, sizeof(*serial), GFP_KERNEL);
	if (!serial)
		return -ENOMEM;
	serial->pdev = pdev;
	platform_set_drvdata(pdev, serial);

	serial->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(serial->regs))
		return PTR_ERR(serial->regs);

	/* power management */
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	/* baud rate config */
	ret = config_baud_rate(serial, pdev);
	if (ret < 0) {
		pm_runtime_disable(&pdev->dev);
		return ret;
	}

	/* soft reset */
	reg_write(serial, UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT, UART_FCR);
	reg_write(serial, 0x00, UART_OMAP_MDR1);

	/* retrival of address, name etc. from the device tree */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		return -1;
	}


	/* interrupts */
	irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(&pdev->dev, irq, serial_interrupt, 0, "serial",
			serial);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register interrupt handler %d\n",
				ret);
		return ret;
	}
	reg_write(serial, UART_IER_RDI, UART_IER);

	/* miscdev pointer madness and registration */
	serial->miscdev.name = devm_kasprintf(&pdev->dev, GFP_KERNEL,
			"serial-%x", res->start);
	serial->miscdev.fops = &serial_fops;
	serial->miscdev.parent = &pdev->dev;
	serial->miscdev.minor = MISC_DYNAMIC_MINOR;
	misc_register(&serial->miscdev);

	return 0;
}

static int serial_remove(struct platform_device *pdev)
{
	struct serial_dev *dev = platform_get_drvdata(pdev);

	misc_deregister(&dev->miscdev);
	/* power management runtime disable */
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
