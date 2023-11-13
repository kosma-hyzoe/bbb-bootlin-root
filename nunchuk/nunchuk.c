// SPDX-License-Identifier: GPL-2.0
#include "linux/mod_devicetable.h"
#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>


/* Add your code here */
int nun_probe(struct i2c_client *client)
{
    pr_alert("Probin'");
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
