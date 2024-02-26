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
#include <linux/miscdevice.h>

#define MISCBEEP_NAME   "miscbeep"
#define MISCBEEP_MINOR  144
#define BEEP_OFF 0
#define BEEP_ON 1   

/* miscbeep设备结构体 */
struct miscbeep_dev{
    struct device_node *nd;     /* 设备节点 */
    int beep_gpio;      /* beep gpio */
};

struct miscbeep_dev miscbeep;

static int miscbeep_open(struct inode *inode,struct file *filp){
    int ret = 0;
    filp->private_data = &miscbeep;

    return ret;
}

static int miscbeep_release(struct inode *inode,struct file *filp){
    int ret = 0;

    return ret;
}

static ssize_t miscbeep_write(struct file *filp,const char __user *buf,size_t count,loff_t *ppos){
    int ret = 0;
    struct miscbeep_dev *dev = filp->private_data;
    unsigned char databuf[1];

    ret = copy_from_user(databuf,buf,count);
    if(ret < 0){
        return -EINVAL;
    }

    if(databuf[0] == BEEP_ON){
        gpio_set_value(dev->beep_gpio,0);
    }else if(databuf[0] == BEEP_OFF){
        gpio_set_value(dev->beep_gpio,1);
    }

    return ret;
}

/* 字符设备操作集 */
struct file_operations miscbeep_fops = {
    .owner = THIS_MODULE,
    .open = miscbeep_open,
    .release = miscbeep_release,
    .write = miscbeep_write,
};

/* miscdevice结构体 */
static struct miscdevice beep_miscdev = {
    .minor = MISCBEEP_MINOR,
    .name = MISCBEEP_NAME,
    .fops = &miscbeep_fops,
};  

/* probe函数 */
static int miscbeep_probe(struct platform_device *dev){
    int ret = 0;
    /* 1.初始化蜂鸣器IO */
    miscbeep.nd = dev->dev.of_node;

    miscbeep.beep_gpio = of_get_named_gpio(miscbeep.nd,"beep-gpios",0);
    if(miscbeep.beep_gpio < 0){
        ret = -EINVAL;
        goto fail_findgpio;
    }

    ret = gpio_request(miscbeep.beep_gpio,"beep-gpio");
    if(ret){
        printk("can't request %d gpio!\r\n",miscbeep.beep_gpio);
        ret = -EINVAL;
        goto fail_findgpio;
    }

    ret = gpio_direction_output(miscbeep.beep_gpio,1);   /* 输出，默认高电平 */
    if(ret < 0){
        goto fail_setoutput;
    }

    /* 2.misc驱动注册 */
    ret = misc_register(&beep_miscdev);
    if(ret < 0){
        goto fail_setoutput;
    }

    return 0;
fail_setoutput:
    gpio_free(miscbeep.beep_gpio);
fail_findgpio:
    return ret;
}

/* remove函数 */
static int miscbeep_remove(struct platform_device *dev){
    gpio_set_value(miscbeep.beep_gpio,1);   /* 拉高，关闭BEEP */
    gpio_free(miscbeep.beep_gpio);  /* 释放gpio */

    misc_deregister(&beep_miscdev);
    
    return 0;
}  

/* platform匹配表 */
static const struct of_device_id beep_of_match[] = {
    {.compatible = "zjf,beep"},
    {/* sentinel */},
};


/* platform */
static struct platform_driver miscbeep_driver = {
    .driver = {
        .name = "imx6ull-beep",
        .of_match_table = beep_of_match,    /* 设备数匹配表 */
    },
    .probe = miscbeep_probe,
    .remove = miscbeep_remove,
};

/* 驱动入口函数 */
static int __init miscbeep_init(void){

    return platform_driver_register(&miscbeep_driver);
}

/* 驱动出口函数 */
static void __exit miscbeep_exit (void){
    platform_driver_unregister(&miscbeep_driver);
}

module_init(miscbeep_init);
module_exit(miscbeep_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhongjunfeng");

