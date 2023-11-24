// SPDX-License-Identifier: GPL-2.0
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include "linux/mod_devicetable.h"
/* Add your code here */

struct serial_dev {
	void __iomem *regs;
};

static int serial_probe(struct platform_device *pdev)
{
	struct serial_dev *serial;

	pr_info("Called %s\n", __func__);

	serial = devm_kzalloc(&pdev->dev, sizeof(*serial), GFP_KERNEL);
	if (!serial)
		return -ENOMEM;

	serial->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(serial->regs))
		return PTR_ERR(serial->regs);

	return 0;
}

static int serial_remove(struct platform_device *pdev)
{
	pr_info("Called %s\n", __func__);
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
                .owner = THIS_MODULE,
		.of_match_table = serial_of_match,
        },
        .probe = serial_probe,
        .remove = serial_remove,
};
module_platform_driver(serial_driver);
MODULE_LICENSE("GPL");
