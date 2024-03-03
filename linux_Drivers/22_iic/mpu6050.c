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
#include <linux/i2c.h>
#include "mpu6050reg.h"

#define MPU6050_CNT 1
#define MPU6050_NAME "mpu6050"

/*  */
struct mpu6050_dev{
    int major;
    int minor;
    dev_t devid;
    struct cdev cdev;
    struct class *class;
    struct device *device;

    void *private_data;
    short acc_x,acc_y,adc_z,gyr_x,gyr_y,gyr_z,temp;
};

static struct mpu6050_dev mpu6050dev;

/* 读取AP3216C的N个寄存器值 */
static int mpu6050_read_regs(struct mpu6050_dev *dev,u8 reg,void *val,int len){
    struct i2c_msg msg[2];
    struct i2c_client *client = (struct i2c_client*)dev->private_data;
    
    /* msg[0] 发送要读取到寄存器首地址*/
    msg[0].addr = client->addr; /* 从机地址，也就诉MPU6050地址 */
    msg[0].flags = 0;   /* 表示为要发送到数据 */
    msg[0].buf = &reg;  /* 要发送的数据，也就是寄存器地址 */
    msg[0].len = 1;     /* 要发送的寄存器地址长度为1 */ 

    /* msg[1] 读取数据*/
    msg[1].addr = client->addr; /* 从机地址，也就诉MPU6050地址 */
    msg[1].flags = I2C_M_RD;   /* 表示读数据 */
    msg[1].buf = val;  /* 接收到的从机发送到数据 */
    msg[1].len = len;     /* 要发送的寄存器地址长度为1 */ 

    return i2c_transfer(client->adapter, msg ,2);
}

/* 向AP3216C写N个寄存器的数据 */
static int mpu6050_write_regs(struct mpu6050_dev *dev, u8 reg, u8 *buf, u8 len){
    u8 b[256];
    struct i2c_msg msg;
    struct i2c_client *client = (struct i2c_client*)dev->private_data;
    
    /* 构建要发送到数据，也就是寄存器首地址+实际到数据 */
    b[0] = reg;
    memcpy(&b[1], buf, len);

    /* msg 发送要读取到寄存器首地址*/
    msg.addr = client->addr; /* 从机地址，也就诉MPU6050地址 */
    msg.flags = 0;           /* 表示为要发送到数据 */
    msg.buf = b;             /* 要发送的数据，也就是寄存器地址+实际数据 */
    msg.len = len + 1;       /* 要发送到数据长度：寄存器地址长度+实际到数据长度 */ 

    return i2c_transfer(client->adapter, &msg ,1);
}

/* 读取AP3216C一个寄存器 */
static unsigned char mpu6050_read_reg(struct mpu6050_dev *dev, u8 reg){
    u8 data = 0;
    
    mpu6050_read_regs(dev,reg,&data,1);

    // i2c_smbus_read_byte_data(dev->private_data,reg);

    return data;
}

/* 向AP3216C一个寄存器写数据 */
static void mpu6050_write_reg(struct mpu6050_dev *dev, u8 reg, u8 data){
    u8 buf = 0;
    buf = data;
    mpu6050_write_regs(dev, reg, &buf, 1);
}

int MPU6050_Init(void){
    mpu6050_write_reg(&mpu6050dev, MPU6050_PWR_MGMT_1, 0x00);       /* 解除休眠状态 */
    mpu6050_write_reg(&mpu6050dev, MPU6050_SMPLRT_DIV, 0x07);       /* 设置陀螺仪采样率为125Hz */
    mpu6050_write_reg(&mpu6050dev, MPU6050_CONFIG, 0x06);           /* 低通滤波器到截止频率为1K，宽带为5Hz */
    mpu6050_write_reg(&mpu6050dev, MPU6050_GYRO_CONFIG, 0x18);      /* 配置陀螺仪量程为2000deg/s，不自检 */
    mpu6050_write_reg(&mpu6050dev, MPU6050_ACCEL_CONFIG, 0x00);     /* 配置加速度计量程为2g，不自检 */

    return mpu6050_read_reg(&mpu6050dev, MPU6050_WHO_AM_I);
}

void MPU6050_Readdata(struct mpu6050_dev *dev){
    char buf[14];
    int i = 0;
    for(i = 0; i < 14; i++ ){
        buf[i] = mpu6050_read_reg(dev, MPU6050_ACCEL_XOUT_H + i);
    }
    dev->acc_x = (buf[0] << 8) | buf[1];
    dev->acc_y = (buf[2] << 8) | buf[3];
    dev->adc_z = (buf[4] << 8) | buf[5];
    dev->temp = (buf[6] << 8) | buf[7];
    dev->gyr_x = (buf[8] << 8) | buf[9];
    dev->gyr_y = (buf[10] << 8) | buf[11];
    dev->gyr_z = (buf[12] << 8) | buf[13];
}

static int mpu6050_open(struct inode *inode, struct file *filp)
{
    int ret = 0;
	filp->private_data = &mpu6050dev; /* 设置私有数据  */

    printk("mpu6050_open!\r\n");

    /* 初始化mpu6050 */
    ret = MPU6050_Init();
    if(ret != 0x68){
        printk("MPU6050_Init err!\r\n");
    }

	return 0;
}

static int mpu6050_release(struct inode *inode,struct file *filp){
    

    printk("mpu6050_release!\r\n");
    return 0;   
}

static int mpu6050_read(struct file *filp, char __user *buf,size_t cnt,loff_t *off){
    long err = 0;
    short data[7];
    struct mpu6050_dev *dev = (struct mpu6050_dev*)filp->private_data;
    printk("mpu6050_read!\r\n");

    /* 向应用返回MPU6050的原始数据 */
    MPU6050_Readdata(dev);
    data[0] = dev->acc_x;
    data[1] = dev->acc_y;
    data[2] = dev->adc_z;
    data[3] = dev->temp;
    data[4] = dev->gyr_x;
    data[5] = dev->gyr_y;
    data[6] = dev->gyr_z;
    
    printk("ACC_X = %d , ACC_Y = %d , ACC_Z = %d\r\n",data[0],data[1],data[2]);
    printk("GYR_X = %d , GYR_Y = %d , GYR_Z = %d\r\n",data[4],data[5],data[6]);
    printk("TEMP = %d \r\n",data[3]);

    err = copy_to_user(buf,data,sizeof(data));

    return 0;
}

/* 设备操作函数 */
static struct file_operations mpu6050_fops = {
	.owner = THIS_MODULE,
	.open = mpu6050_open,
	.read = mpu6050_read,
    .release = mpu6050_release,
};

static int mpu6050_probe(struct i2c_client *client,const struct i2c_device_id *id){
    int ret = 0;
    printk("mpu6050_probe!\r\n");
    /* 搭建字符设备驱动框架，在/dev/ */
    /* 注册字符设备驱动 */
	/*1、创建设备号 */
    mpu6050dev.major = 0;/* 由系统分配主设备号 */
	if (mpu6050dev.major) {		/*  定义了设备号 */
		mpu6050dev.devid = MKDEV(mpu6050dev.major, 0);
		ret = register_chrdev_region(mpu6050dev.devid, MPU6050_CNT, MPU6050_NAME);
	} else {						/* 没有定义设备号 */
		ret = alloc_chrdev_region(&mpu6050dev.devid, 0, MPU6050_CNT, MPU6050_NAME);	/* 申请设备号 */
		mpu6050dev.major = MAJOR(mpu6050dev.devid);	/* 获取分配号的主设备号 */
        mpu6050dev.minor = MINOR(mpu6050dev.devid);
	}
    if(ret < 0){
        printk("chrdev_region err!\r\n");
        goto fail_devid;
    }
    printk("major = %d,minor = %d\r\n",mpu6050dev.major,mpu6050dev.minor);
	
	/* 2、初始化cdev */
	mpu6050dev.cdev.owner = THIS_MODULE;
	cdev_init(&mpu6050dev.cdev, &mpu6050_fops);
	
	/* 3、添加一个cdev */
	ret = cdev_add(&mpu6050dev.cdev, mpu6050dev.devid, MPU6050_CNT);
    if(ret < 0){
        goto fail_cdev;
    }

	/* 4、创建类 */
	mpu6050dev.class = class_create(THIS_MODULE, MPU6050_NAME);
	if (IS_ERR(mpu6050dev.class)) {
		ret = PTR_ERR(mpu6050dev.class);
        goto fail_class;
	}

	/* 5、创建设备 */
	mpu6050dev.device = device_create(mpu6050dev.class, NULL, mpu6050dev.devid, NULL, MPU6050_NAME);
	if (IS_ERR(mpu6050dev.device)) {
		ret = PTR_ERR(mpu6050dev.device);
        goto fail_device;
	}

    mpu6050dev.private_data = client;

    return 0;

fail_device:
    class_destroy(mpu6050dev.class);
fail_class:
    cdev_del(&mpu6050dev.cdev);
fail_cdev:
    unregister_chrdev_region(mpu6050dev.devid,MPU6050_CNT);
fail_devid:
    return ret;
}

static int mpu6050_remove(struct i2c_client *client){
    cdev_del(&mpu6050dev.cdev);
    unregister_chrdev_region(mpu6050dev.devid,MPU6050_CNT);
    device_destroy(mpu6050dev.class, mpu6050dev.devid);
    class_destroy(mpu6050dev.class);
    return 0;
}

/* 传统的匹配表 */
static struct i2c_device_id mpu6050_id[] = {
    {"zjf,mpu6050",0},
    {}
};

/* 设备树的匹配表 */
static struct of_device_id mpu6050_of_match[] = {
    { .compatible = "zjf,mpu6050"},
    {}
};

/* i2c_driver */
static struct i2c_driver mpu6050_driver = {
    .probe = mpu6050_probe,
    .remove = mpu6050_remove,
    .driver = {
        .name = "mpu6050",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(mpu6050_of_match),
    },
    .id_table = mpu6050_id,
}; 

/* 入口 */
static int __init mpu6050_init(void){
    int ret = 0;

    ret = i2c_add_driver(&mpu6050_driver);

    return ret;
}

/* 出口 */
static void __exit mpu6050_exit(void){
    i2c_del_driver(&mpu6050_driver);
}

/* 注册驱动和卸载驱动 */
module_init(mpu6050_init);
module_exit(mpu6050_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhongjunfeng");

