#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define BEEP_CNT  1
#define BEEP_NAME "beep"
#define BEEPOFF      0
#define BEEPON       1

/* beep设备结构体 */
struct beep_dev
{
    dev_t devid;
    int major;
    int minor;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    struct device_node *nd;
    int beep_gpio;
};

struct beep_dev beep;

static int beep_open(struct inode *inode,struct file *filp){
    filp->private_data = &beep;
    return 0;
}

static int beep_release(struct inode *inode,struct file *filp){

    return 0;
}

static ssize_t beep_write(struct file *filp,const char __user *buf,
            size_t count,loff_t *ppos)
{
    int ret ;
    
    unsigned char databuf[1];
    struct beep_dev *dev = filp->private_data;

    ret = copy_from_user(databuf,buf,count);
    if(ret < 0){
        return -EFAULT;
    }

    if(databuf[0] == BEEPOFF){
        gpio_set_value(dev->beep_gpio,1);
    }else if(databuf[0] == BEEPON)
    {
        gpio_set_value(dev->beep_gpio,0);
    }

    return 0;
}

static const struct file_operations beep_fops = {
    .owner      =   THIS_MODULE,
    .write      =   beep_write,
    .open       =   beep_open,
    .release    =   beep_release,
};

/* 入口 */
static int __init beep_init(void){
    int ret = 0;

    /* 1.注册字符设备驱动 */
    beep.major = 0;
    if(beep.major){
        beep.devid = MKDEV(beep.major,0);
        ret = register_chrdev_region(beep.devid,BEEP_CNT,BEEP_NAME);
    }else {
        ret = alloc_chrdev_region(&beep.devid,0,BEEP_CNT,BEEP_NAME);
        beep.major = MAJOR(beep.devid);
        beep.minor = MINOR(beep.devid);
    }
    if(ret < 0){
        goto fail_devid;
    }
    printk("beep major = %d ,minor = %d \r\n",beep.major,beep.minor);

    /* 2.初始化cdev */
    beep.cdev.owner = THIS_MODULE;
    cdev_init(&beep.cdev,&beep_fops);

    /* 3.添加cdev */
    ret = cdev_add(&beep.cdev,beep.devid,BEEP_CNT);
    if(ret < 0){
        goto fail_cdev;
    }

    /* 4.创建类 */
    beep.class = class_create(THIS_MODULE,BEEP_NAME);
    if(IS_ERR(beep.class)){
        ret =  PTR_ERR(beep.class);
        goto fail_class;
    }

    /* 5.创建设备 */
    beep.device  = device_create(beep.class, NULL, beep.devid, NULL, BEEP_NAME);
    if(IS_ERR(beep.device)){
        ret =  PTR_ERR(beep.device);
        goto fail_device;
    }

    /* 初始化BEEP */
    beep.nd = of_find_node_by_path("/beep");
    if(beep.nd == NULL){
        ret = -EINVAL;
        goto fail_nd;
    }

    /* 获取IO */
    beep.beep_gpio = of_get_named_gpio(beep.nd,"beep-gpios",0);
    if(beep.beep_gpio < 0){
        ret = -EINVAL;
        goto fail_nd;
    }

    /* 请求IO */
    ret = gpio_request(beep.beep_gpio,"beep-gpio");
    if(ret){
        printk("Can't request beep gpio\r\n");
        goto fail_nd;
    }

    ret = gpio_direction_output(beep.beep_gpio,0);
    if(ret < 0){
        goto fail_set;
    }

    gpio_set_value(beep.beep_gpio,1);

    return 0;

fail_set:
    gpio_free(beep.beep_gpio);
fail_nd:
    device_destroy(beep.class,beep.devid);
fail_device:
    class_destroy(beep.class);
fail_class:
    cdev_del(&beep.cdev);
fail_cdev:
    unregister_chrdev_region(beep.devid,BEEP_CNT);
fail_devid:
    return ret;
}

/* 出口 */
static void __exit beep_exit(void){
    /* 关闭beep */
    gpio_set_value(beep.beep_gpio,1);

    /* 注销字符设备驱动 */
    cdev_del(&beep.cdev);

    /* 释放设备号 */
    unregister_chrdev_region(beep.devid,BEEP_CNT);

    device_destroy(beep.class,beep.devid);

    class_destroy(beep.class);

    gpio_free(beep.beep_gpio);

}

/* 注册驱动和卸载驱动 */
module_init(beep_init);
module_exit(beep_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhongjunfeng");

