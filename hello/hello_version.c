// SPDX-License-Identifier: GPL-2.0
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/utsname.h>

static char *whom = "my dear";
static time64_t seconds;

module_param(whom, charp, 0644);

static int __init hello_init(void)
{
	pr_alert("Hello %s! You are currently using Linux %s\n", whom,
			utsname()->release);
	seconds = ktime_get_seconds();
	return 0;
}

static void __exit hello_exit(void)
{
	pr_alert("Been running for %llds. Goodbye :3\n",
			ktime_get_seconds() - seconds);
}

module_init(hello_init);
module_exit(hello_exit);
MODULE_LICENSE("GPL");
