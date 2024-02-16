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

#define DTSLED_CNT  1    //
#define DTSLED_NAME "dtsled"

/* 地址映射后的虚拟地址指针 */
static void __iomem *IMX6U_CCM_CCGR1;
static void __iomem *SW_MUX_GPIO1_IO03;
static void __iomem *SW_PAD_GPIO1_IO03;
static void __iomem *GPIO1_DR;
static void __iomem *GPIO1_GDIR;

#define LEDOFF  0    /*关闭*/
#define LEDON   1    /*打开*/

/* dtsled设备结构体 */
struct dtsled_dev
{
    dev_t devid;            /* 设备号 */
    struct cdev cdev;       /* 字符设备 */
    struct class *class;    /* 类 */
    struct device *device;  /* 设备 */
    int major;              /* 主设备号 */
    int minor;              /* 次设备号 */
    struct device_node *nd; /* 设备节点 */

};

struct dtsled_dev dtsled;

static void led_switch(u8 sta){
    u32 val = 0;
    if(sta == LEDON){
        val = readl(GPIO1_DR);
        val &= ~(1 << 3);         /* bit3清零，打开led灯*/
        writel(val,GPIO1_DR);
    }
    else if(sta == LEDOFF){
       val = readl(GPIO1_DR);
        val |= 1 << 3;         /* bit3置1，关闭led灯*/
        writel(val,GPIO1_DR); 
    }
}

static int dtsled_open(struct inode *inode,struct file *filp){
    filp->private_data = &dtsled;
    return 0;
}

static int dtsled_release(struct inode *inode,struct file *filp){
    struct dtsled_dev *dev = (struct dtsled_dev*)filp->private_data;

    return 0;
}

static ssize_t dtsled_write(struct file *filp,const char __user *buf,
            size_t count,loff_t *ppos)
{
    struct dtsled_dev *dev = (struct dtsled_dev*)filp->private_data;

    int retvalue;
    unsigned char databuf[1];

    retvalue = copy_from_user(databuf,buf,count);
    if(retvalue < 0){
        return -EFAULT;
    }

    /*判断开灯还是关灯*/
    led_switch(databuf[0]);

    return 0;
}

/* 字符设备操作集 */
static const struct file_operations dtsled_fops = 
{
    .owner = THIS_MODULE,
    .write = dtsled_write,
    .open = dtsled_open,
    .release = dtsled_release,
};


/* 入口 */
static int __init dtsled_init(void){
    int ret = 0;
    const char *str;
    u32 regdata[10];
    u8 i = 0;
    unsigned int val = 0;

    /* 注册字符设备 */
    /* 1.申请设备号 */
    dtsled.major = 0;   /**/
    if(dtsled.major){
        dtsled.devid = MKDEV(dtsled.major,0);
        ret = register_chrdev_region(dtsled.devid,DTSLED_CNT,DTSLED_NAME);
    }
    else{
        ret = alloc_chrdev_region(&dtsled.devid,0,DTSLED_CNT,DTSLED_NAME);
        dtsled.major = MAJOR(dtsled.devid);
        dtsled.minor = MINOR(dtsled.minor);
    }
    if(ret < 0){
        goto faile_devid;
    }

    /* 2.添加字符设备 */
    dtsled.cdev.owner = THIS_MODULE;
    cdev_init(&dtsled.cdev,&dtsled_fops);
    ret = cdev_add(&dtsled.cdev,dtsled.devid,DTSLED_CNT);
    if(ret < 0){
        goto fail_cdev;
    }

    /* 3.自动创建设备节点 */
    dtsled.class = class_create(THIS_MODULE,DTSLED_NAME);
    if(IS_ERR(dtsled.class)){
        ret = PTR_ERR(dtsled.class);
        goto fail_class;
    }

    dtsled.device = device_create(dtsled.class,NULL,dtsled.devid,NULL,DTSLED_NAME);
    if(IS_ERR(dtsled.device)){
        ret = PTR_ERR(dtsled.device);
        goto fail_device;
    }

    /* 获取设备树属性内容 */
    dtsled.nd = of_find_node_by_path("/zjfled");
    if(dtsled.nd == NULL){
        ret = -EINVAL;
        goto fail_findnd;
    }

    /* 获取属性 */
    ret = of_property_read_string(dtsled.nd,"status",&str);
    if(ret < 0){
        goto fail_rs;
    }else{
        printk("status = %s\r\n",str);
    }

    ret = of_property_read_string(dtsled.nd,"compatible",&str);
    if(ret < 0){
        goto fail_rs;
    }else{
        printk("compatible = %s\r\n",str);
    }
    
    #if 0
    ret = of_property_read_u32_array(dtsled.nd,"reg",regdata,10);
    if(ret < 0){
        goto fail_rs;
    }else{
        printk("reg data:\r\n");
        for(i = 0; i < 10; i++){
            printk("%#X ",regdata[i]);
        }
        printk("\r\n");
    }

    /* LED灯初始化 */
    /*1.初始化LED*/
 
    IMX6U_CCM_CCGR1 = ioremap(regdata[0],regdata[1]);
    SW_MUX_GPIO1_IO03 = ioremap(regdata[2],regdata[3]);
    SW_PAD_GPIO1_IO03 = ioremap(regdata[4],regdata[5]);
    GPIO1_DR = ioremap(regdata[6],regdata[7]);
    GPIO1_GDIR = ioremap(regdata[8],regdata[9]);
    #endif
    IMX6U_CCM_CCGR1 = of_iomap(dtsled.nd,0);
    SW_MUX_GPIO1_IO03 = of_iomap(dtsled.nd,1);
    SW_PAD_GPIO1_IO03 = of_iomap(dtsled.nd,2);
    GPIO1_DR = of_iomap(dtsled.nd,3);
    GPIO1_GDIR = of_iomap(dtsled.nd,4);

    /* 1.2,初始化*/
    val = readl(IMX6U_CCM_CCGR1);
    val &= ~(3 << 26);      /* 先清除以前到配置bit26，27*/
    val |= 3 << 26;         /* bit26,27置1 */
    writel(val,IMX6U_CCM_CCGR1);

    writel(0x5,SW_MUX_GPIO1_IO03);      /* 设置复用 */
    writel(0x10b0,SW_PAD_GPIO1_IO03);   /* 设置电气属性 */

    val = readl(GPIO1_GDIR);
    val |= 1 << 3;         /* bit3置1 ,设置为输出*/
    writel(val,GPIO1_GDIR);

    val = readl(GPIO1_DR);
    val |= (1 << 3);         /* bit3置1，关闭led灯*/
    writel(val,GPIO1_DR);


    return 0;


fail_rs:
fail_findnd:
    device_destroy(dtsled.class,dtsled.devid);
fail_device:
    class_destroy(dtsled.class);
fail_class:
    cdev_del(&dtsled.cdev);
fail_cdev:
    unregister_chrdev_region(dtsled.devid,DTSLED_CNT);
faile_devid:
    return ret;
}

/* 出口 */
static void __exit dtsled_exit(void){
    unsigned int val = 0;
    /* 关灯 */
    val = readl(GPIO1_DR);
    val |= 1 << 3;         /* bit3置1，关闭led灯*/
    writel(val,GPIO1_DR);

    /*1，取消地址映射*/
    iounmap(IMX6U_CCM_CCGR1);
    iounmap(SW_MUX_GPIO1_IO03);
    iounmap(SW_PAD_GPIO1_IO03);
    iounmap(GPIO1_DR);
    iounmap(GPIO1_GDIR);

    /* 删除设备号 */
    cdev_del(&dtsled.cdev);
    /* 释放设备号 */
    unregister_chrdev_region(dtsled.devid,DTSLED_CNT);

    /* 摧毁设备 */
    device_destroy(dtsled.class,dtsled.devid);
    /* 摧毁类 */
    class_destroy(dtsled.class);
}

/* 注册驱动和卸载驱动 */
module_init(dtsled_init);
module_exit(dtsled_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhongjunfeng");

