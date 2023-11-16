// SPDX-License-Identifier: GPL-2.0

#include "linux/device.h"
#include "linux/mod_devicetable.h"
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>


#define INIT_BYTES_1 {0xf0, 0x55}
#define INIT_BYTES_2 {0xfb, 0x00}
#define READ_BYTES {0x00}
#define DEV_NAME "nunchuk"

#define PRESSED 0
#define RELEASED 1

static char read_buf[6];

int nun_read_regs(struct i2c_client *client)
{
	int ret_val;
	char rb[] = READ_BYTES;

	usleep_range(10000, 20000);

	ret_val = i2c_master_send(client, rb, sizeof(rb));
	if (ret_val < 0) {
		pr_alert(DEV_NAME ": error %d while sending bytes starting with %c\n",
				ret_val, rb[0]);
		return ret_val;
	}
	usleep_range(10000, 20000);

	ret_val = i2c_master_recv(client, read_buf, 6);
	if (ret_val < 0) {
		pr_alert(DEV_NAME ": i2c_master_recv: error %d\n", ret_val);
		return ret_val;
	} else if (ret_val == 0) {
		pr_alert(DEV_NAME ": Zero bytes read!");
		return 1;
	}
	return 0;
}

int nun_probe(struct i2c_client *client)
{
	int ret_val, i;
	const char ib1[2] = INIT_BYTES_1;
	const char ib2[2] = INIT_BYTES_2;

	char button_state_byte;
	int z_state, c_state;

	struct input_dev *input;

	pr_alert(DEV_NAME ": device detected");

	/* Initialization */
	ret_val = i2c_master_send(client, ib1, sizeof(ib1));
	if (ret_val < 0) {
		pr_alert("error %d while sending bytes starting with %c\n",
		 ret_val, ib1[0]);
		return ret_val;
	}

	udelay(1000);

	ret_val = i2c_master_send(client, ib2, sizeof(ib2));
	if (ret_val < 0) {
		pr_alert("error %d while sending bytes starting with %c\n",
		 ret_val, ib2[0]);
		return ret_val;
	}

	if (!(input = devm_input_allocate_device(&client->dev))) {
		return 1;
	}


	/* Reading button states */
	for (i = 0; i < 2; i++)
		nun_read_regs(client);

	button_state_byte = read_buf[5];
	z_state = button_state_byte & 0b01 ? RELEASED : PRESSED;
	c_state = button_state_byte & 0b10 ? RELEASED : PRESSED;

	pr_alert("Z: %s\n", z_state == PRESSED ? "pressed" : "released");
	pr_alert("C: %s\n", c_state == PRESSED ? "pressed" : "released");

	return 0;
}

int nun_remove(struct i2c_client *client)
{
	pr_alert("Removin'");
	return 0;
}

static const struct i2c_device_id nunchuk_id[] = {
	{DEV_NAME},
	{  }
};



static const struct of_device_id nunchuk_of_match[] = {
	{ .compatible = "nintendo," DEV_NAME },
	{ }
};

struct i2c_driver nun_driver = {
	.driver = {
		.name = DEV_NAME,
		.of_match_table = nunchuk_of_match,
	},
	.probe_new = nun_probe,
	.remove = nun_remove,
	.id_table = nunchuk_id,
};


static struct file_operations fops = {
	.owner = THIS_MODULE,
	/* .open = nun_open, */
	/* .release = nun_close, */
	/* .read = nun_read, */
	/* .write = nun_write */
};

MODULE_DEVICE_TABLE(i2c, nunchuk_id);
module_i2c_driver(nun_driver);

MODULE_LICENSE("GPL");
