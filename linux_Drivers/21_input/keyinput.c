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

#define KEYINPUT_CNT      1
#define KEYINPUT_NAME    "keyinput"
#define KEY_NUM           1
#define KEY0VALUE          0X01
#define INVAKEY             0XFF

/* key 结构体 */
struct irq_keydesc{
    int gpio;               /* io编号 */
    int irqnum;             /* 中断号 */
    unsigned int value;    /* 键值 */
    char name[10];          /* 名字 */
    irqreturn_t (*handler)(int,void *);/* 中断处理函数 */   
};

/* imx6uirq设备结构体 */
struct keyinput_dev
{
    struct device_node *nd;
    struct irq_keydesc irqkey[KEY_NUM];
    struct timer_list timer;

    struct input_dev *inputdev;     /* 输入设备 */
};

struct keyinput_dev keyinputdev;

/*中断处理函数*/
static irqreturn_t key0_handler(int irq, void *dev_id){
    struct keyinput_dev *dev =dev_id;

    dev->timer.data = (volatile unsigned long)dev_id;
    mod_timer(&dev->timer, jiffies + msecs_to_jiffies(50)); /* 10ms定时 */

    return IRQ_HANDLED;
}

/* 定时器回调函数 */
void timer_func(unsigned long arg){
    int value = 0;
    struct keyinput_dev *dev = (struct keyinput_dev*)arg;
    
    // printk("timer_func\r\n");
    value = gpio_get_value(dev->irqkey[0].gpio);
    if(value == 0){     /* 按下 */
        /* 上报按键值 */
        input_event(dev->inputdev,EV_KEY,BTN_0,1);
    }else if(value == 1){       /* 释放 */
        /* 上报按键值 */
        input_event(dev->inputdev,EV_KEY,BTN_0,0);
    }
    input_sync(dev->inputdev);
}

/* 按键初始化 */
static int keyio_init(struct keyinput_dev *dev){
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
    dev->irqkey[0].value = BTN_0;
    /*2.按键中断初始化*/
    for(i = 0;i<KEY_NUM;i++){
        ret = request_irq(dev->irqkey[i].irqnum,dev->irqkey[i].handler,
                            IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                            dev->irqkey[i].name,&keyinputdev);
        if(ret){
            printk("irq %d request failed!\r\n",dev->irqkey[i].irqnum);
            goto fail_irq;
        }
    }

    /* 3.初始化定时器 */
    init_timer(&keyinputdev.timer);
    keyinputdev.timer.function = timer_func;

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
static int __init keyinput_init(void){
    int ret = 0;

    /* 1.初始化IO */
    ret = keyio_init(&keyinputdev);
    if(ret < 0){
        goto fail_keyinit;
    }

    /* 2.注册input_dev */
    keyinputdev.inputdev = input_allocate_device();
    if(keyinputdev.inputdev == NULL){
        ret = -EINVAL;
        goto fail_keyinit;
    }

    /* 设置 */
    keyinputdev.inputdev->name = KEYINPUT_NAME;
    __set_bit(EV_KEY,keyinputdev.inputdev->evbit);  /* 按键事件 */
    __set_bit(EV_REP,keyinputdev.inputdev->evbit);  /* 重复事件 */
    __set_bit(BTN_0,keyinputdev.inputdev->keybit);  /* 按键值 */

    /* 注册 */
    ret = input_register_device(keyinputdev.inputdev);
    if(ret){
        goto fail_input_register;
    }

    return 0;
fail_input_register:
    input_free_device(keyinputdev.inputdev);
fail_keyinit:
    return ret;
}

/* 出口 */
static void __exit keyinput_exit(void){
    int i = 0;

    /* 1.释放中断 */
    for(i = 0;i<KEY_NUM;i++){
        free_irq(keyinputdev.irqkey[i].irqnum,&keyinputdev);
    }

    /* 2.释放io */
    for(i = 0;i<KEY_NUM;i++){
        gpio_free(keyinputdev.irqkey[i].gpio);
    }

    /* 3.删除定时器 */
    del_timer_sync(&keyinputdev.timer);		/* 删除timer */

    /* 注销input_dev */
    input_unregister_device(keyinputdev.inputdev);
    input_free_device(keyinputdev.inputdev);
}

/* 注册驱动和卸载驱动 */
module_init(keyinput_init);
module_exit(keyinput_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhongjunfeng");

