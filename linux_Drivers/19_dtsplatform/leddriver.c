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
#include <linux/poll.h>
#include <linux/fcntl.h>
#include <linux/platform_device.h>

#define GPIOLED_CNT  1
#define GPIOLED_NAME "dtsplatled"
#define LEDOFF      0
#define LEDON       1

/* gpioled设备结构体 */
struct gpioled_dev
{
    dev_t devid;
    int major;
    int minor;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    struct device_node *nd;
    int led_gpio;
};

struct gpioled_dev gpioled;

static int led_open(struct inode *inode,struct file *filp){
    filp->private_data = &gpioled;
    return 0;
}

static int led_release(struct inode *inode,struct file *filp){

    return 0;
}

static ssize_t led_write(struct file *filp,const char __user *buf,
            size_t count,loff_t *ppos)
{
    int ret ;
    unsigned char databuf[1];
    struct gpioled_dev *dev = filp->private_data;

    ret = copy_from_user(databuf,buf,count);
    if(ret < 0){
        return -EINVAL;
    }

    if(databuf[0] == LEDON){
        gpio_set_value(dev->led_gpio,0);
    }else if(databuf[0] == LEDOFF){
        gpio_set_value(dev->led_gpio,1);
    }

    return 0;
}

static const struct file_operations led_fops = {
    .owner      =   THIS_MODULE,
    .write      =   led_write,
    .open       =   led_open,
    .release    =   led_release,
};

static int led_probe(struct platform_device *dev){
	// printk("led probe\r\n");
	int ret = 0;

    /* 注册字符设备驱动 */
    gpioled.major = 0;
    if(gpioled.major){
        gpioled.devid = MKDEV(gpioled.major,0);
        ret = register_chrdev_region(gpioled.devid,GPIOLED_CNT,GPIOLED_NAME);
    }else {
        ret = alloc_chrdev_region(&gpioled.devid,0,GPIOLED_CNT,GPIOLED_NAME);
        gpioled.major = MAJOR(gpioled.devid);
        gpioled.minor = MINOR(gpioled.devid);
    }
    if(ret < 0){
        goto fail_devid;
    }
    printk("gpioled major = %d ,minor = %d \r\n",gpioled.major,gpioled.minor);

    /* 初始化cdev */
    gpioled.cdev.owner = THIS_MODULE;
    cdev_init(&gpioled.cdev,&led_fops);

    /* 添加cdev */
    ret = cdev_add(&gpioled.cdev,gpioled.devid,GPIOLED_CNT);
    if(ret < 0){
        goto fail_cdev;
    }

    /* 创建类 */
    gpioled.class = class_create(THIS_MODULE,GPIOLED_NAME);
    if(IS_ERR(gpioled.class)){
        ret = PTR_ERR(gpioled.class);
        goto fail_class;
    }

    /* 创建设备 */
    gpioled.device  = device_create(gpioled.class, NULL, gpioled.devid, NULL, GPIOLED_NAME);
    if(IS_ERR(gpioled.device)){
        ret = PTR_ERR(gpioled.device);
        goto fail_device;
    }

    /* 获取设备节点 */
	#if 0
    gpioled.nd = of_find_node_by_path("/gpioled");
    if(gpioled.nd == NULL){
        ret = -EINVAL;
        goto fail_findnode;
    }
	#endif
	gpioled.nd = dev->dev.of_node;	/*  */	

    /* 获取led对应到GPIO */
    gpioled.led_gpio = of_get_named_gpio(gpioled.nd,"led-gpios",0);
    if(gpioled.led_gpio < 0){
        printk("can't find led gpio\r\n");
        ret = -EINVAL;
        goto fail_findnode;
    }
    printk("led gpio num = %d\r\n",gpioled.led_gpio);

    /* 申请IO */
    ret = gpio_request(gpioled.led_gpio,"led-gpio");
    if(ret){
        printk("Failed to request the led gpio\r\n");
        ret = -EINVAL;
        goto fail_findnode;
    }

    /* 使用IO，设置为输出 */
    ret = gpio_direction_output(gpioled.led_gpio,1);
    if(ret){
        goto fail_setoutput;
    }

    /* 输出LED低电平，点亮LED灯 */
    gpio_set_value(gpioled.led_gpio,0);

    return 0;

fail_setoutput:
    gpio_free(gpioled.led_gpio);
fail_findnode:
    device_destroy(gpioled.class,gpioled.devid);
fail_device:
    class_destroy(gpioled.class);
fail_class:
    cdev_del(&gpioled.cdev);
fail_cdev:
    unregister_chrdev_region(gpioled.devid,GPIOLED_CNT);
fail_devid:
    return ret;
}

static int led_remove(struct platform_device *dev){
	// printk("led remove\r\n");
	 /* 关led */
    gpio_set_value(gpioled.led_gpio,1);

    /* 注销字符设备驱动 */
    cdev_del(&gpioled.cdev);

    /*  */
    unregister_chrdev_region(gpioled.devid,GPIOLED_CNT);

    device_destroy(gpioled.class,gpioled.devid);

    class_destroy(gpioled.class);

    /* 释放IO */
    gpio_free(gpioled.led_gpio);
	
	return 0;
}

struct of_device_id led_of_match[] = {
	{.compatible = "zjf,gpioled"},
	{/* sentinel */},
};

struct platform_driver led_driver = {
	.driver = {
		.name = "imx6ull-led",		/* 无设备数和设备进行匹配，驱动名字 */
		.of_match_table = led_of_match,	/* 设备树匹配表 */
	},
	.probe = led_probe,
	.remove = led_remove,
};


/* 
    驱动加载
*/
static int __init leddriver_init(void){
    
    
    return platform_driver_register(&led_driver);
}

/*
    驱动卸载
*/
static void __exit leddriver_exit(void){
	platform_driver_unregister(&led_driver);
}

module_init(leddriver_init);
module_exit(leddriver_exit);
MODULE_LICENSE("GPL");
MODULE_ALIAS("zhongjunfeng");

