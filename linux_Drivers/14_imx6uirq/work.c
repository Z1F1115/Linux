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

#define IMX6UIRQ_CNT      1
#define IMX6UIRQ_NAME    "imx6uirq"
#define KEY_NUM           1
#define KEY0VALUE          0X01
#define INVAKEY             0XFF

/* key 结构体 */
struct irq_keydesc{
    int gpio;               /* io编号 */
    int irqnum;             /* 中断号 */
    unsigned char value;    /* 键值 */
    char name[10];          /* 名字 */
    irqreturn_t (*handler)(int,void *);/* 中断处理函数 */   

    struct tasklet_struct tasklet;
    // struct work_struct work;
};

/* imx6uirq设备结构体 */
struct imx6uirq_dev
{
    dev_t devid;
    int major;
    int minor;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    struct device_node *nd;
    struct irq_keydesc irqkey[KEY_NUM];
    struct timer_list timer;

    atomic_t keyvalue;
    atomic_t releasekey;

    struct work_struct work;
};

struct imx6uirq_dev imx6uirq;

static int imx6uirq_open(struct inode *inode,struct file *filp){
    filp->private_data = &imx6uirq;
    return 0;
}

static int imx6uirq_read(struct file *filp, char __user *buf,size_t count,loff_t *ppos){
    int ret = 0;
    unsigned char keyvalue;
    unsigned char releasekey;
    struct imx6uirq_dev *dev = filp->private_data;

    keyvalue = atomic_read(&dev->keyvalue);
    releasekey = atomic_read(&dev->releasekey);

    if(releasekey){ /* 有效按键 */
        if(keyvalue & 0x80){
            keyvalue &= ~0x80;
            ret = copy_to_user(buf,&keyvalue,sizeof(keyvalue));
        }else{
            goto data_error;
        }
        atomic_set(&dev->releasekey,0); /* 按下标志清零 */
    }else{
        goto data_error;
    }

    return ret;
data_error:
    return -EINVAL;
}



static const struct file_operations imx6uirq_fops = {
    .owner      =   THIS_MODULE,
    .open       =   imx6uirq_open,
    .read       =   imx6uirq_read,
};

/*中断处理函数*/
static irqreturn_t key0_handler(int irq, void *dev_id){
    struct imx6uirq_dev *dev = dev_id;

#if 0
    dev->timer.data = (unsigned long)dev_id;
    mod_timer(&dev->timer, jiffies + msecs_to_jiffies(50)); /* 10ms定时 */
#endif

    // tasklet_schedule(&dev->irqkey[0].tasklet);

    schedule_work(&dev->work);

    return IRQ_HANDLED;
}

/* tasklet */
static void key_tasklet(unsigned long data){
    struct imx6uirq_dev *dev = (struct imx6uirq_dev*)data;
    

    dev->timer.data = data;
    mod_timer(&dev->timer, jiffies + msecs_to_jiffies(50)); /* 10ms定时 */

}

/* work */
static void key_work(struct work_struct *work){
    // struct imx6uirq_dev *dev = (struct imx6uirq_dev*)data;

    struct imx6uirq_dev *dev = container_of(work,struct imx6uirq_dev,work);

    dev->timer.data = (unsigned long)dev;
    mod_timer(&dev->timer, jiffies + msecs_to_jiffies(50)); /* 10ms定时 */

}

/* 定时器回调函数 */
void timer_func(unsigned long arg){
    int value = 0;
    struct imx6uirq_dev *dev = (struct imx6uirq_dev*)arg;
    
    // printk("timer_func\r\n");
    value = gpio_get_value(dev->irqkey[0].gpio);
    if(value == 0){     /* 按下 */
        // printk("KEY0 Push!\r\n");
        atomic_set(&dev->keyvalue,dev->irqkey[0].value);
    }else if(value == 1){       /* 释放 */
        // printk("KEY0 Release!\r\n");
        atomic_set(&dev->keyvalue, 0x80 | (dev->irqkey[0].value));
        atomic_set(&dev->releasekey, 1);
    }
}

/* 按键初始化 */
static int keyio_init(struct imx6uirq_dev *dev){
    int ret = 0;
    int i = 0;
    
    /*1.按键初始化*/
    dev->nd = of_find_node_by_path("/key");
    if(dev->nd == NULL){
        ret = -EINVAL;
        goto fail_nd;
    }

    for(i = 0;i<KEY_NUM;i++){
        dev->irqkey[i].gpio = of_get_named_gpio(dev->nd,"key-gpios",i); 
        if(dev->irqkey[i].gpio < 0){
            ret = -EINVAL;
            goto fail_gpio;
        }
    }

    for(i = 0;i<KEY_NUM;i++){
        memset(dev->irqkey[i].name,0,sizeof(dev->irqkey[i].name));
        sprintf(dev->irqkey[i].name, "KEY%d", i);
        ret = gpio_request(dev->irqkey[i].gpio,dev->irqkey[i].name);
        if(ret){
            ret = -EBUSY;
            printk("IO %d can't requset!\r\n",dev->irqkey[i].gpio);
            goto fail_request;
        }
        ret = gpio_direction_input(dev->irqkey[i].gpio);
        if(ret < 0){
            ret = -EINVAL;
            goto fail_input;
        }
        
        dev->irqkey[i].irqnum = gpio_to_irq(dev->irqkey[i].gpio);/* 获取中断号 */
        // dev->irqkey[i].irqnum = irq_of_parse_and_map(dev->nd,i);

    }

    dev->irqkey[0].handler = key0_handler;
    dev->irqkey[0].value = KEY0VALUE;
    // dev->irqkey[0].tasklet = key_tasklet;
    /*2.按键中断初始化*/
    for(i = 0;i<KEY_NUM;i++){
        ret = request_irq(dev->irqkey[i].irqnum,dev->irqkey[i].handler,
                            IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                            dev->irqkey[i].name,&imx6uirq);
        if(ret){
            printk("irq %d request failed!\r\n",dev->irqkey[i].irqnum);
            goto fail_irq;
        }

        // tasklet_init(&dev->irqkey[i].tasklet,key_tasklet,(unsigned long )dev);
        // INIT_WORK(&dev->irqkey[i].work,key_work);
    }

    INIT_WORK(&dev->work,key_work);

    /* 3.初始化定时器 */
    init_timer(&imx6uirq.timer);
    imx6uirq.timer.function = timer_func;

    return 0;
fail_irq:
fail_input:
    for(i = 0;i<KEY_NUM;i++){
        gpio_free(dev->irqkey[i].gpio);
    }
fail_request:
fail_gpio:
fail_nd:
    return ret;
}

/* 入口 */
static int __init imx6uirq_init(void){
    int ret = 0;

    /* 1.注册字符设备驱动 */
    imx6uirq.major = 0;
    if(imx6uirq.major){
        imx6uirq.devid = MKDEV(imx6uirq.major,0);
        ret = register_chrdev_region(imx6uirq.devid,IMX6UIRQ_CNT,IMX6UIRQ_NAME);
    }else {
        ret = alloc_chrdev_region(&imx6uirq.devid,0,IMX6UIRQ_CNT,IMX6UIRQ_NAME);
        imx6uirq.major = MAJOR(imx6uirq.devid);
        imx6uirq.minor = MINOR(imx6uirq.devid);
    }
    if(ret < 0){
        goto fail_devid;
    }
    printk("imx6uirq major = %d ,minor = %d \r\n",imx6uirq.major,imx6uirq.minor);

    /* 2.初始化cdev */
    imx6uirq.cdev.owner = THIS_MODULE;
    cdev_init(&imx6uirq.cdev,&imx6uirq_fops);

    /* 3.添加cdev */
    ret = cdev_add(&imx6uirq.cdev,imx6uirq.devid,IMX6UIRQ_CNT);
    if(ret < 0){
        goto fail_cdev;
    }

    /* 4.创建类 */
    imx6uirq.class = class_create(THIS_MODULE,IMX6UIRQ_NAME);
    if(IS_ERR(imx6uirq.class)){
        ret =  PTR_ERR(imx6uirq.class);
        goto fail_class;
    }

    /* 5.创建设备 */
    imx6uirq.device  = device_create(imx6uirq.class, NULL, imx6uirq.devid, NULL, IMX6UIRQ_NAME);
    if(IS_ERR(imx6uirq.device)){
        ret =  PTR_ERR(imx6uirq.device);
        goto fail_device;
    }

    /* 初始化IO */
    ret = keyio_init(&imx6uirq);
    if(ret < 0){
        goto fail_keyinit;
    }

    /* 初始化原子变量 */
    atomic_set(&imx6uirq.keyvalue,INVAKEY);
    atomic_set(&imx6uirq.releasekey,0);

    return 0;
fail_keyinit:
fail_device:
    class_destroy(imx6uirq.class);
fail_class:
    cdev_del(&imx6uirq.cdev);
fail_cdev:
    unregister_chrdev_region(imx6uirq.devid,IMX6UIRQ_CNT);
fail_devid:
    return ret;
}

/* 出口 */
static void __exit imx6uirq_exit(void){
    int i = 0;

    /* 1.释放中断 */
    for(i = 0;i<KEY_NUM;i++){
        free_irq(imx6uirq.irqkey[i].irqnum,&imx6uirq);
    }

    /* 3.删除定时器 */
    del_timer_sync(&imx6uirq.timer);		/* 删除timer */

    /* 注销字符设备驱动 */
    cdev_del(&imx6uirq.cdev);

    /* 释放设备号 */
    unregister_chrdev_region(imx6uirq.devid,IMX6UIRQ_CNT);

    device_destroy(imx6uirq.class,imx6uirq.devid);

    class_destroy(imx6uirq.class);

    /* 2.释放io */
    for(i = 0;i<KEY_NUM;i++){
        gpio_free(imx6uirq.irqkey[i].gpio);
    }

}

/* 注册驱动和卸载驱动 */
module_init(imx6uirq_init);
module_exit(imx6uirq_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhongjunfeng");

