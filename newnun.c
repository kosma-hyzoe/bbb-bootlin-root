/* SPDX-License-Identifier: GPL-2.0 */

#include "linux/mod_devicetable.h"
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>

#define INIT_BYTES_1                                                           \
	{                                                                      \
		0xf0, 0x55                                                     \
	}
#define INIT_BYTES_2                                                           \
	{                                                                      \
		0xfb, 0x00                                                     \
	}
#define READ_BYTE 0x00

#define PRESSED 0
#define RELEASED 1

static char read_buf[6];

int _print_bytes_read(void)
{
	pr_alert("%d\n", read_buf[5]);
	return 0;
}

int _nun_read_regs(struct i2c_client *client)
{
	int ret_val;
	char byte[] = {READ_BYTE};

	usleep_range(10000, 20000);
	ret_val = i2c_master_send(client, byte, sizeof(byte));
	if (ret_val < 0) {
		pr_alert("error %d while sending bytes starting with %c\n",
			 ret_val, READ_BYTE);
		return ret_val;
	}
	usleep_range(10000, 20000);

	ret_val = i2c_master_recv(client, read_buf, 6);
	if (ret_val < 0) {
		pr_alert("i2c_master_recv: error %d\n", ret_val);
		return ret_val;
	} else if (ret_val == 0) {
		pr_alert("Zero bytes read!");
		return 1;
	}
	_print_bytes_read();
	return 0;
}

int nun_probe(struct i2c_client *client)
{
	int ret_val, i;
	const char ib1[2] = INIT_BYTES_1;
	const char ib2[2] = INIT_BYTES_2;

	char button_state_byte;
	int z_state, c_state;

	pr_alert("Nunchuk device detected");

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

	/* Reading button states */
	for (i = 0; i < 2; i++)
		_nun_read_regs(client);

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

static const struct i2c_device_id nunchuk_id[] = {{"nunchuk"}, {}};

MODULE_DEVICE_TABLE(i2c, nunchuk_id);

static const struct of_device_id nunchuk_of_match[] = {
    {.compatible = "nintendo,nunchuk"}, {}};

struct i2c_driver nun_driver = {
    .driver =
	{
	    .name = "nunchuk",
	    .of_match_table = nunchuk_of_match,
	},
    .probe_new = nun_probe,
    .remove = nun_remove,
    .id_table = nunchuk_id,
};

MODULE_LICENSE("GPL");
module_i2c_driver(nun_driver);
