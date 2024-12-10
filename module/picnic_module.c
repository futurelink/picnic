#include <linux/init.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_platform.h>
#endif

#include "picnic_main.h"
#include "picnic_dev.h"

extern picnic_dev_t *picnic_dev;

/****************************************************************************
 *                       KERNEL MODULE INITIALIZATION
 ****************************************************************************/

#ifdef CONFIG_OF
static const struct of_device_id picnic_of_match[] = {
    { .compatible = "futurelink,picnic", },
    { /* End of list */ }
};
MODULE_DEVICE_TABLE(of, picnic_of_match);
#endif

static struct platform_driver picnic_driver = {
    .probe = picnic_probe,
    .driver = {
	.name = "picnic",
	.owner = THIS_MODULE,
	.of_match_table = of_match_ptr(picnic_of_match),
    }
};
//module_platform_driver(picnic_driver);

// Custom init and exit methods
static int __init picnic_init(void) {
    if (platform_driver_register(&picnic_driver) != 0) {
        printk(KERN_INFO "PiCNC driver failed\n");
	return -1;
    }

    printk(KERN_INFO "PiCNC driver loaded\n");
    return 0;
}

static void __exit picnic_exit(void) {
    if(picnic_dev != 0) picnic_device_deinit(picnic_dev);
    platform_driver_unregister(&picnic_driver);
    printk(KERN_INFO "PiCNC driver unloaded\n");
}

module_init(picnic_init);
module_exit(picnic_exit);
