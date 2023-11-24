// SPDX-License-Identifier: GPL-2.0

#include "linux/device.h"
#include "linux/gfp.h"
#include "linux/mod_devicetable.h"
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>

#define N_INIT_BYTES 2
#define INIT_BYTES_1 {0xf0, 0x55}
#define INIT_BYTES_2 {0xfb, 0x00}

#define N_INIT_READ_BYTES 1
#define N_BYTES_TO_READ 6
#define BUTTON_STATE_INDEX 5
#define INIT_READ_BYTES {0x00}

#define DEV_NAME "nunchuk"
#define DELAY 10000
#define DELAY_MAX 20000
#define POLL_INTERVAL 50

#define PRESSED 1
#define RELEASED 0

static char rx_buf[6];
static int z_state, c_state;

struct nunchuk_dev {
	struct i2c_client *i2c_client;
};


int _nun_read_regs(struct i2c_client *client)
{
	int ret_val;
	char tx_buf[] = INIT_READ_BYTES;

	usleep_range(DELAY, DELAY_MAX);

	ret_val = i2c_master_send(client, tx_buf, sizeof(tx_buf));
	if (ret_val != N_INIT_READ_BYTES) {
		pr_alert(DEV_NAME ":i2c_master_send: error %d\n", ret_val);
		return ret_val;
	}
	usleep_range(DELAY, DELAY_MAX);

	ret_val = i2c_master_recv(client, rx_buf, N_BYTES_TO_READ);
	if (ret_val != N_BYTES_TO_READ) {
		pr_alert(DEV_NAME ": i2c_master_recv: error %d\n", ret_val);
		return ret_val;
	}
	return 0;

}

void nun_poll(struct input_dev *input)
{
	int ret_val;
	char button_state_byte;

	struct nunchuk_dev *nun = input_get_drvdata(input);
	struct i2c_client *client = nun->i2c_client;

	ret_val = _nun_read_regs(client);
	if (ret_val != N_BYTES_TO_READ)
		return;

	button_state_byte = rx_buf[BUTTON_STATE_INDEX];
	z_state = button_state_byte & 0b01 ? RELEASED : PRESSED;
	c_state = button_state_byte & 0b10 ? RELEASED : PRESSED;

	input_report_key(input, BTN_Z, z_state);
	input_report_key(input, BTN_C, c_state);
	input_sync(input);
}


int nun_probe(struct i2c_client *client)
{
	int ret_val;
	const char tx_buf1[2] = INIT_BYTES_1;
	const char tx_buf2[2] = INIT_BYTES_2;

	struct input_dev *input;
	struct nunchuk_dev *nun;


	/* Initialization */
	ret_val = i2c_master_send(client, tx_buf1, sizeof(tx_buf1));
	if (ret_val != N_INIT_BYTES) {
		pr_alert("error %d while sending bytes starting with %c\n",
				ret_val, tx_buf1[0]);
		return ret_val;
	}

	udelay(DELAY);

	ret_val = i2c_master_send(client, tx_buf2, sizeof(tx_buf2));
	if (ret_val != N_INIT_BYTES) {
		pr_alert("error %d while sending bytes starting with %c\n",
				ret_val, tx_buf2[0]);
		return ret_val;
	}

	/* Allocation */
	input = devm_input_allocate_device(&client->dev);
	if (!input) {
		pr_alert("failed to allocate input device");
		return -ENOMEM;
	}
	nun =  devm_kzalloc(&client->dev, sizeof(*nun), GFP_KERNEL);
	if (!nun)
		return -ENOMEM;

	/* Input subsystem registration */
	nun->i2c_client = client;
	input_set_drvdata(input, nun);

	input->name = "Wii Nunchuck";
	input->id.bustype = BUS_I2C;

	set_bit(EV_KEY, input->evbit);
	set_bit(BTN_C, input->keybit);
	set_bit(BTN_Z, input->keybit);

	ret_val = input_setup_polling(input, nun_poll);
	if (ret_val) {
		dev_err(&client->dev, "input setup polling: (%d)\n", ret_val);
		return ret_val;
	}
	input_set_poll_interval(input, POLL_INTERVAL);
	input_set_min_poll_interval(input, POLL_INTERVAL);
	input_set_max_poll_interval(input, POLL_INTERVAL);

	ret_val = input_register_device(input);
	if (ret_val < 0) {
		pr_alert("failed to register input device");
		return ret_val;
	}
	return 0;
}

int nun_remove(struct i2c_client *client)
{
	/* Nothing to do here */
	return 0;
}

static const struct i2c_device_id nunchuk_id[] = {
	{ DEV_NAME },
	{  }
};

MODULE_DEVICE_TABLE(i2c, nunchuk_id);

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


module_i2c_driver(nun_driver);

MODULE_LICENSE("GPL");
