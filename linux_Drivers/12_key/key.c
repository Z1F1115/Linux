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

#define KEY_CNT      1
#define KEY_NAME    "key"
#define KEY0VALUE   0XF0
#define INVAKEY     0X00

/* key设备结构体 */
struct key_dev
{
    dev_t devid;
    int major;
    int minor;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    struct device_node *nd;
    int key_gpio;
    atomic_t keyvalue;
};

struct key_dev key;

static int key_open(struct inode *inode,struct file *filp){
    filp->private_data = &key;
    return 0;
}

static int key_release(struct inode *inode,struct file *filp){

    return 0;
}

static ssize_t key_write(struct file *filp,const char __user *buf,
            size_t count,loff_t *ppos)
{
    int ret = 0;
    
    

    return ret;
}

static ssize_t key_read(struct file *filp, char __user *buf,size_t count,loff_t *ppos){
    int value;
    struct key_dev *dev = filp->private_data;
    int ret = 0;

    if(gpio_get_value(dev->key_gpio) == 0){/*  按下 */
        while(!gpio_get_value(dev->key_gpio));//等待释放
        atomic_set(&dev->keyvalue,KEY0VALUE);
    }else{
        atomic_set(&dev->keyvalue,INVAKEY);
    }

    value = atomic_read(&dev->keyvalue);

    ret = copy_to_user(buf,&value,sizeof(value));

    return ret;
}

static const struct file_operations key_fops = {
    .owner      =   THIS_MODULE,
    .write      =   key_write,
    .read       =   key_read,
    .open       =   key_open,
    .release    =   key_release,
};

/* key io初始化 */
static int keyio_init(struct key_dev *dev){
    int ret = 0;

    dev->nd = of_find_node_by_path("/key");
    if(dev->nd == NULL){
        ret = -EINVAL;
        goto fail_nd;
    }

    dev->key_gpio = of_get_named_gpio(dev->nd,"key-gpios",0);
    if(dev->key_gpio < 0){
        ret = -EINVAL;
        goto fail_gpio;
    }

    ret = gpio_request(dev->key_gpio,"key0");
    if(ret){
        ret = -EBUSY;
        printk("IO %d cant't request\r\n",dev->key_gpio);
        goto fail_request;
    }

    ret = gpio_direction_input(dev->key_gpio);
    if(ret < 0){
        ret = -EINVAL;
        goto fail_input;
    }

    return 0;

fail_input:
    gpio_free(dev->key_gpio);
fail_request:
fail_gpio:
fail_nd:
    return ret;
}

/* 入口 */
static int __init key_init(void){
    int ret = 0;

    /* 初始化atomic */
    atomic_set(&key.keyvalue,INVAKEY);

    /* 1.注册字符设备驱动 */
    key.major = 0;
    if(key.major){
        key.devid = MKDEV(key.major,0);
        ret = register_chrdev_region(key.devid,KEY_CNT,KEY_NAME);
    }else {
        ret = alloc_chrdev_region(&key.devid,0,KEY_CNT,KEY_NAME);
        key.major = MAJOR(key.devid);
        key.minor = MINOR(key.devid);
    }
    if(ret < 0){
        goto fail_devid;
    }
    printk("key major = %d ,minor = %d \r\n",key.major,key.minor);

    /* 2.初始化cdev */
    key.cdev.owner = THIS_MODULE;
    cdev_init(&key.cdev,&key_fops);

    /* 3.添加cdev */
    ret = cdev_add(&key.cdev,key.devid,KEY_CNT);
    if(ret < 0){
        goto fail_cdev;
    }

    /* 4.创建类 */
    key.class = class_create(THIS_MODULE,KEY_NAME);
    if(IS_ERR(key.class)){
        ret =  PTR_ERR(key.class);
        goto fail_class;
    }

    /* 5.创建设备 */
    key.device  = device_create(key.class, NULL, key.devid, NULL, KEY_NAME);
    if(IS_ERR(key.device)){
        ret =  PTR_ERR(key.device);
        goto fail_device;
    }

    ret = keyio_init(&key);
    if(ret < 0){
        goto fail_device;
    }

    return 0;


fail_device:
    class_destroy(key.class);
fail_class:
    cdev_del(&key.cdev);
fail_cdev:
    unregister_chrdev_region(key.devid,KEY_CNT);
fail_devid:
    return ret;
}

/* 出口 */
static void __exit key_exit(void){
    
    /* 注销字符设备驱动 */
    cdev_del(&key.cdev);

    /* 释放设备号 */
    unregister_chrdev_region(key.devid,KEY_CNT);

    device_destroy(key.class,key.devid);

    class_destroy(key.class);

    gpio_free(key.key_gpio);

}

/* 注册驱动和卸载驱动 */
module_init(key_init);
module_exit(key_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhongjunfeng");

