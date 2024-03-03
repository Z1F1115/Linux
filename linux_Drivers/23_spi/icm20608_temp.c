#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/input.h>
#include <linux/spi/spi.h>

#define ICM20608_CNT 1
#define ICM20608_NAME "icm20608"

/* 设备结构体 */
struct icm20608_dev{
    int major;
    int minor;
    dev_t devid;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    void *private_data;

};

static struct icm20608_dev icm20608dev;

static int icm20608_open(struct inode *inode, struct file *filp)
{
	filp->private_data = &icm20608dev; /* 设置私有数据  */

	return 0;
}

static int icm20608_read(struct file *filp, char __user *buf,size_t cnt,loff_t *off){
   

    return 0;
}

static int icm20608_release(struct inode *inode,struct file *filp){
    
    return 0;   
}


/* 设备操作函数 */
static const struct file_operations icm20608_fops = {
	.owner = THIS_MODULE,
	.open = icm20608_open,
	.read = icm20608_read,
    .release = icm20608_release,
};

static int icm20608_probe(struct spi_device *spi){
    int ret = 0;
    printk("icm20608_probe\r\n");

    /* 搭建字符设备驱动框架，在/dev/ */
    /* 注册字符设备驱动 */
	/*1、创建设备号 */
    icm20608dev.major = 0;/* 由系统分配主设备号 */
	if (icm20608dev.major) {		/*  定义了设备号 */
		icm20608dev.devid = MKDEV(icm20608dev.major, 0);
		ret = register_chrdev_region(icm20608dev.devid, ICM20608_CNT, ICM20608_NAME);
	} else {						/* 没有定义设备号 */
		ret = alloc_chrdev_region(&icm20608dev.devid, 0, ICM20608_CNT, ICM20608_NAME);	/* 申请设备号 */
		icm20608dev.major = MAJOR(icm20608dev.devid);	/* 获取分配号的主设备号 */
        icm20608dev.minor = MINOR(icm20608dev.devid);
	}
    if(ret < 0){
        printk("chrdev_region err!\r\n");
        goto fail_devid;
    }
    printk("major = %d,minor = %d\r\n",icm20608dev.major,icm20608dev.minor);
	
    /* 注册字符设备 */
	/* 2、初始化cdev */
	icm20608dev.cdev.owner = THIS_MODULE;
	cdev_init(&icm20608dev.cdev, &icm20608_fops);
	
	/* 3、添加一个cdev */
	ret = cdev_add(&icm20608dev.cdev, icm20608dev.devid, ICM20608_CNT);
    if(ret < 0){
        goto fail_cdev;
    }

	/* 4、自动创建设备节点 */
	icm20608dev.class = class_create(THIS_MODULE, ICM20608_NAME);
	if (IS_ERR(icm20608dev.class)) {
		ret = PTR_ERR(icm20608dev.class);
        goto fail_class;
	}

	/* 5、创建设备 */
	icm20608dev.device = device_create(icm20608dev.class, NULL, icm20608dev.devid, NULL, ICM20608_NAME);
	if (IS_ERR(icm20608dev.device)) {
		ret = PTR_ERR(icm20608dev.device);
        goto fail_device;
	}

    /* 设置icm20608dev的私有数据 */
    icm20608dev.private_data = spi;

    return 0;

fail_device:
    class_destroy(icm20608dev.class);
fail_class:
    cdev_del(&icm20608dev.cdev);
fail_cdev:
    unregister_chrdev_region(icm20608dev.devid,ICM20608_CNT);
fail_devid:
    return ret;
}

static int icm20608_remove(struct spi_device *spi){
    int ret = 0;
    cdev_del(&icm20608dev.cdev);
    unregister_chrdev_region(icm20608dev.devid,ICM20608_CNT);
    device_destroy(icm20608dev.class, icm20608dev.devid);
    class_destroy(icm20608dev.class);
    return ret;
}

/* 传统的匹配表 */
static struct spi_device_id icm20608_id[] = {
    {"zjf,icm20608",0},
    {}
};


/* 设备树匹配 */
static struct of_device_id icm20608_of_match[] = {
    { .compatible = "zjf,icm20608"},
    { }
};

/* spi_driver */
static struct spi_driver icm20608_driver = {
    .probe = icm20608_probe,
    .remove = icm20608_remove,
    .driver = {
        .name = "icm20608",
        .owner = THIS_MODULE,
        .of_match_table = icm20608_of_match,
    },
    .id_table = icm20608_id,
};

/* 入口 */
static int __init icm20608_init(void){
    int ret = 0;

    ret = spi_register_driver(&icm20608_driver);

    return ret;
}

/* 出口 */
static void __exit icm20608_exit(void){
    spi_unregister_driver(&icm20608_driver);
}

/* 注册驱动和卸载驱动 */
module_init(icm20608_init);
module_exit(icm20608_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhongjunfeng");

