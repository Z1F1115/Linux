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
#include <linux/timer.h>
#include <linux/jiffies.h>

#define TIMER_CNT      1
#define TIMER_NAME    "timer"

#define CLOSE_CMD       _IO(0XEF,1)        //关闭命令
#define OPEN_CMD        _IO(0XEF,2)        //打开命令
#define SETPERIOD_CMD   _IOW(0XEF,3,int)   //设置周期

/* timer设备结构体 */
struct timer_dev
{
    dev_t devid;
    int major;
    int minor;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    struct device_node *nd;
    int led_gpio; 
    struct timer_list timer;    /* 定时器 */
    int timeperiod;             /* 定时周期ms */    
    spinlock_t lock;
};

struct timer_dev timerdev;

static int timer_open(struct inode *inode,struct file *filp){
    filp->private_data = &timerdev;
    return 0;
}

static int timer_release(struct inode *inode,struct file *filp){

    return 0;
}

static long timer_ioctl(struct file *filp,unsigned int cmd,unsigned long arg){
    struct timer_dev *dev = filp->private_data;
    int ret = 0;
    int value = 0;
    int timerperiod;
	unsigned long flags;

    switch(cmd){
        case CLOSE_CMD:
            del_timer_sync(&dev->timer);
            break;
        case OPEN_CMD:
            spin_lock_irqsave(&dev->lock, flags);
			timerperiod = dev->timeperiod;
			spin_unlock_irqrestore(&dev->lock, flags);
            mod_timer(&dev->timer,jiffies + (msecs_to_jiffies(timerperiod)));
            break;
        case SETPERIOD_CMD:
            ret = copy_from_user(&value,(int *)arg,sizeof(int));
            if(ret < 0){
                return -EFAULT;
            }

            spin_lock_irqsave(&dev->lock, flags);
			dev->timeperiod = value;
			spin_unlock_irqrestore(&dev->lock, flags);
            mod_timer(&dev->timer,jiffies + (msecs_to_jiffies(value)));
            break;
        
    }

    return ret;
}


static const struct file_operations timer_fops = {
    .owner      =   THIS_MODULE,
    .open       =   timer_open,
    .unlocked_ioctl = timer_ioctl,
    .release    =   timer_release,
};

/* 定时器处理函数 */
static void timer_func(unsigned long arg){
    struct timer_dev *dev = (struct timer_dev*)arg;
    static int sta = 1;
    int timerperiod;
	unsigned long flags;

    sta = !sta;
    gpio_set_value(dev->led_gpio,sta);

    /* 重启定时器 */
	spin_lock_irqsave(&dev->lock, flags);
	timerperiod = dev->timeperiod;
	spin_unlock_irqrestore(&dev->lock, flags);
    mod_timer(&dev->timer,jiffies + (msecs_to_jiffies(timerperiod)));
}

/* 初始化LED */
int led_init(struct timer_dev *dev){
    int ret = 0;

    dev->nd = of_find_node_by_path("/gpioled");
    if(dev->nd == NULL){
        ret = -EINVAL;
        goto fail_nd;
    }

    dev->led_gpio = of_get_named_gpio(dev->nd,"led-gpios",0);
    if(dev->led_gpio < 0){
        ret = -EINVAL;
        goto fail_gpio;
    }

    ret = gpio_request(dev->led_gpio,"led");
    if(ret){
        ret = -EBUSY;
        printk("IO %d can't requset!\r\n",dev->led_gpio);
        goto fail_request;
    }

    ret = gpio_direction_output(dev->led_gpio,1);
    if(ret < 0){
        ret = -EINVAL;
        goto fail_gpioset;
    }

    return 0;
fail_gpioset:
    gpio_free(dev->led_gpio);
fail_request:
fail_gpio:
fail_nd:
    return ret;
}


/* 入口 */
static int __init timer_init(void){
    int ret = 0;

    /* 初始化自旋锁 */
	spin_lock_init(&timerdev.lock);

    /* 1.注册字符设备驱动 */
    timerdev.major = 0;
    if(timerdev.major){
        timerdev.devid = MKDEV(timerdev.major,0);
        ret = register_chrdev_region(timerdev.devid,TIMER_CNT,TIMER_NAME);
    }else {
        ret = alloc_chrdev_region(&timerdev.devid,0,TIMER_CNT,TIMER_NAME);
        timerdev.major = MAJOR(timerdev.devid);
        timerdev.minor = MINOR(timerdev.devid);
    }
    if(ret < 0){
        goto fail_devid;
    }
    printk("timerdev major = %d ,minor = %d \r\n",timerdev.major,timerdev.minor);

    /* 2.初始化cdev */
    timerdev.cdev.owner = THIS_MODULE;
    cdev_init(&timerdev.cdev,&timer_fops);

    /* 3.添加cdev */
    ret = cdev_add(&timerdev.cdev,timerdev.devid,TIMER_CNT);
    if(ret < 0){
        goto fail_cdev;
    }

    /* 4.创建类 */
    timerdev.class = class_create(THIS_MODULE,TIMER_NAME);
    if(IS_ERR(timerdev.class)){
        ret =  PTR_ERR(timerdev.class);
        goto fail_class;
    }

    /* 5.创建设备 */
    timerdev.device  = device_create(timerdev.class, NULL, timerdev.devid, NULL, TIMER_NAME);
    if(IS_ERR(timerdev.device)){
        ret =  PTR_ERR(timerdev.device);
        goto fail_device;
    }


    /* 6.初始化led灯 */
    ret = led_init(&timerdev);
    if(ret < 0){
        goto fail_ledinit;
    }

    /* 7.初始化定时器 */
    init_timer(&timerdev.timer);

    timerdev.timeperiod = 500;
    timerdev.timer.expires = jiffies + (msecs_to_jiffies(timerdev.timeperiod));
    timerdev.timer.function = timer_func;
    timerdev.timer.data = (unsigned long )&timerdev;
    /* 添加到系统 */
    // add_timer(&timerdev.timer);

    return 0;

fail_ledinit:
fail_device:
    class_destroy(timerdev.class);
fail_class:
    cdev_del(&timerdev.cdev);
fail_cdev:
    unregister_chrdev_region(timerdev.devid,TIMER_CNT);
fail_devid:
    return ret;
}

/* 出口 */
static void __exit timer_exit(void){

    /* 关灯 */
    gpio_set_value(timerdev.led_gpio,1);
    
    /* 删除定时器 */
    del_timer(&timerdev.timer);

    /* 注销字符设备驱动 */
    cdev_del(&timerdev.cdev);

    /* 释放设备号 */
    unregister_chrdev_region(timerdev.devid,TIMER_CNT);

    device_destroy(timerdev.class,timerdev.devid);

    class_destroy(timerdev.class);

    /* 释放io */
    gpio_free(timerdev.led_gpio);

}

/* 注册驱动和卸载驱动 */
module_init(timer_init);
module_exit(timer_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhongjunfeng");

