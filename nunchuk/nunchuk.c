// SPDX-License-Identifier: GPL-2.0
#include "linux/mod_devicetable.h"
#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>

#define INIT_BYTES {0xf0, 0x55}


/* Add your code here */
int nun_probe(struct i2c_client *client)
{
    int ret_val;
    char bytes[] = INIT_BYTES;

    pr_alert("Probin'");
    if (ret_val = i2c_master_send(client, bytes , 2))
        pr_alert("i2c_master_send: error %d\n", ret_val);

    return 0;
}

int nun_remove(struct i2c_client *client)
{
    pr_alert("Removin'");
    return 0;
}

static const struct i2c_device_id nunchuk_id[] = {
    {"nunchuk"},
    {  }
};

MODULE_DEVICE_TABLE(i2c, nunchuk_id);

static const struct of_device_id nunchuk_of_match[] = {
    { .compatible = "nintendo,nunchuk" },
    { }
};



struct i2c_driver nun_driver = {
    .driver = {
        .name = "nunchuk",
        .of_match_table = nunchuk_of_match,
    },
    .probe_new = nun_probe,
    .remove = nun_remove,
    .id_table = nunchuk_id,
};

MODULE_LICENSE("GPL");
module_i2c_driver(nun_driver);
