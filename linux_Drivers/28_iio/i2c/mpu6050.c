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
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/unaligned/be_byteshift.h>
#include "mpu6050reg.h"

#define MPU6050_CNT 1
#define MPU6050_NAME "mpu6050"
#define MPU6050_TEMP_OFFSET	         -521
#define MPU6050_TEMP_SCALE		     340000000

/*
 * mpu6050陀螺仪分辨率，对应250、500、1000、2000，计算方法：
 * 以正负250度量程为例，500/2^16=0.007629，扩大1000000倍，就是7629
 */
static const int gyro_scale_mpu6050[] = {7629, 15258, 30517, 61035};

/* 
 * mpu6050加速度计分辨率，对应2、4、8、16 计算方法：
 * 以正负2g量程为例，4/2^16=0.000061035，扩大1000000000倍，就是61035
 */
static const int accel_scale_mpu6050[] = {61035, 122070, 244140, 488281};

/* 扫描元素 */
enum inv_mpu6050_scan {
    INV_MPU6050_SCAN_ACCL_X,
	INV_MPU6050_SCAN_ACCL_Y,
	INV_MPU6050_SCAN_ACCL_Z,
	INV_MPU6050_SCAN_TEMP,
	INV_MPU6050_SCAN_GYRO_X,
	INV_MPU6050_SCAN_GYRO_Y,
	INV_MPU6050_SCAN_GYRO_Z,
	INV_MPU6050_SCAN_TIMESTAMP,
};

#define MPU6050_CHAN(_type, _channel2, _index)                          \
    {                                                                   \
        .type = _type,                                                  \
        .modified = 1,                                                  \
        .channel2 = _channel2,                                          \
        .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),           \
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),                   \
        .scan_index = _index,                                           \
        .scan_type = {                                                  \
                .sign = 's',                                            \
                .realbits = 16,                                         \
                .storagebits = 16,                                      \
                .shift = 0,                                             \
                .endianness = IIO_BE,                                   \
                },                                                      \
    }

/* mpu6050通道 */
static const struct iio_chan_spec mpu6050_channels[] ={
    /* 温度 */
    {
        .type = IIO_TEMP,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW)
                | BIT(IIO_CHAN_INFO_OFFSET)
                | BIT(IIO_CHAN_INFO_SCALE),
        .scan_index = INV_MPU6050_SCAN_TEMP,
        .scan_type = {
                .sign = 's',
                .realbits = 16,
                .storagebits = 16,
                .endianness = IIO_BE,
        },
    },

    /* 加速度X，Y，Z三个通道 */
    MPU6050_CHAN(IIO_ACCEL, IIO_MOD_X, INV_MPU6050_SCAN_ACCL_X),
    MPU6050_CHAN(IIO_ACCEL, IIO_MOD_Y, INV_MPU6050_SCAN_ACCL_Y),
    MPU6050_CHAN(IIO_ACCEL, IIO_MOD_Z, INV_MPU6050_SCAN_ACCL_Z),

    /* 陀螺仪X，Y，Z三个通道 */
    MPU6050_CHAN(IIO_ANGL_VEL, IIO_MOD_X, INV_MPU6050_SCAN_GYRO_X),
    MPU6050_CHAN(IIO_ANGL_VEL, IIO_MOD_Y, INV_MPU6050_SCAN_GYRO_Y),
    MPU6050_CHAN(IIO_ANGL_VEL, IIO_MOD_Z, INV_MPU6050_SCAN_GYRO_Z),
};

/* 设备结构体 */
struct mpu6050_dev{
    struct i2c_client *client;
    struct mutex lock;
    struct regmap *regmap;
    struct regmap_config regmap_config;
};

/* 读取mpu6050一个寄存器 */
static unsigned char mpu6050_read_reg(struct mpu6050_dev *dev, u8 reg){
    unsigned int data = 0;

    regmap_read(dev->regmap,reg,&data);

    return (u8)data;
}

/* 向mpu6050一个寄存器写数据 */
static void mpu6050_write_reg(struct mpu6050_dev *dev, u8 reg, u8 data){
    
    regmap_write(dev->regmap,reg,data);
}

int MPU6050_Init(struct mpu6050_dev *dev){
    mpu6050_write_reg(dev, MPU6050_PWR_MGMT_1, 0x00);       /* 解除休眠状态 */
    mpu6050_write_reg(dev, MPU6050_SMPLRT_DIV, 0x07);       /* 设置陀螺仪采样率为125Hz */
    mpu6050_write_reg(dev, MPU6050_CONFIG, 0x06);           /* 低通滤波器到截止频率为1K，宽带为5Hz */
    mpu6050_write_reg(dev, MPU6050_GYRO_CONFIG, 0x18);      /* 配置陀螺仪量程为2000deg/s，不自检 */
    mpu6050_write_reg(dev, MPU6050_ACCEL_CONFIG, 0x00);     /* 配置加速度计量程为2g，不自检 */

    return mpu6050_read_reg(dev, MPU6050_WHO_AM_I);
}

static int mpu6050_sensor_show(struct mpu6050_dev *dev, int reg,
                    int axis, int *val){
    int ind, result;
    __be16 d;
    
    ind = (axis - IIO_MOD_X) * 2;
    result = regmap_bulk_read(dev->regmap, reg + ind, (u8 *)&d, 2);
    if(result){
        return -EINVAL;
    }
    *val = (short)be16_to_cpup(&d);
    
    return IIO_VAL_INT;
}

static int mpu6050_read_channle_data(struct iio_dev *indio_dev,
                            struct iio_chan_spec const *chan, int *val){
    int ret = 0;
    struct mpu6050_dev *dev = iio_priv(indio_dev);

    switch (chan->type)
    {
    case IIO_ACCEL:
        ret = mpu6050_sensor_show(dev, MPU6050_ACCEL_XOUT_H, chan->channel2, val);
        break;
    case IIO_ANGL_VEL:
        ret = mpu6050_sensor_show(dev, MPU6050_GYRO_XOUT_H, chan->channel2, val);
        break;
    case IIO_TEMP:
        ret = mpu6050_sensor_show(dev, MPU6050_TEMP_OUT_H, IIO_MOD_X, val);
        break;   
    default:
        ret = -EINVAL;
        break;
    }
    return ret;
}

static int mpu6050_write_gyro_scale(struct mpu6050_dev *dev, int val){
    int result, i;
    u8 d;

    for (i = 0; i < ARRAY_SIZE(gyro_scale_mpu6050); i++)
    {
        if (gyro_scale_mpu6050[i] == val)
        {
            d = (i << 3);
            result = regmap_write(dev->regmap, MPU6050_GYRO_CONFIG, d);
            if(result){
                return result;
            }
            return 0;
        }
    }
    
    return -EINVAL;
}

static int mpu6050_write_accel_scale(struct mpu6050_dev *dev, int val){
    int result, i;
    u8 d;

    for (i = 0; i < ARRAY_SIZE(accel_scale_mpu6050); i++)
    {
        if (accel_scale_mpu6050[i] == val)
        {
            d = (i << 3);
            result = regmap_write(dev->regmap, MPU6050_ACCEL_CONFIG, d);
            if(result){
                return result;
            }
            return 0;
        }
    }
    
    return -EINVAL;
}

static int mpu6050_read_raw(struct iio_dev *indio_dev,
        struct iio_chan_spec const *chan, int *val, int *val2,
        long mask){
    int ret = 0;
    struct mpu6050_dev *dev = iio_priv(indio_dev);
    unsigned char regdata = 0;

    /* 区分读取到数据类型，是RAW？SCALE？ */
    switch (mask)
    {
    case IIO_CHAN_INFO_RAW:
        mutex_lock(&dev->lock);
        ret = mpu6050_read_channle_data(indio_dev, chan, val);
        mutex_unlock(&dev->lock);
        return ret;
    case IIO_CHAN_INFO_SCALE:
        switch (chan->type)
        {
        case IIO_ANGL_VEL:
            mutex_lock(&dev->lock);
            regdata  = (mpu6050_read_reg(dev,MPU6050_GYRO_CONFIG) & 0x18) >> 3;
            *val = 0;
            *val2 = gyro_scale_mpu6050[regdata];
            mutex_unlock(&dev->lock);
            return IIO_VAL_INT_PLUS_MICRO;
        case IIO_ACCEL:
            mutex_lock(&dev->lock);
            regdata  = (mpu6050_read_reg(dev, MPU6050_ACCEL_CONFIG) & 0x18) >> 3;
            *val = 0;
            *val2 = accel_scale_mpu6050[regdata];
            mutex_unlock(&dev->lock);
            return IIO_VAL_INT_PLUS_NANO;
        case IIO_TEMP:
            *val = MPU6050_TEMP_SCALE/ 1000000;
			*val2 = MPU6050_TEMP_SCALE % 1000000;
            return IIO_VAL_INT_PLUS_MICRO;
        default:
            return -EINVAL;
        }
        return ret;    
    case IIO_CHAN_INFO_OFFSET:
        switch (chan->type)
        {
        case IIO_TEMP:
            *val = MPU6050_TEMP_OFFSET;
            return IIO_VAL_INT;
        default:
            return -EINVAL;
        }
        return ret;   
    default:
        return -EINVAL;
    }
    
    return 0;
}

static int mpu6050_write_raw(struct iio_dev *indio_dev,
        struct iio_chan_spec const *chan, int val, int val2, long mask){
    int ret = 0;
    struct mpu6050_dev *dev = iio_priv(indio_dev);

    switch (mask)
    {
    case IIO_CHAN_INFO_SCALE:
        switch (chan->type)
        {
        case IIO_ANGL_VEL:
            mutex_lock(&dev->lock);
            ret = mpu6050_write_gyro_scale(dev, val2);
            mutex_unlock(&dev->lock);
            break;
        case IIO_ACCEL:
            mutex_lock(&dev->lock);
            ret = mpu6050_write_accel_scale(dev, val2);
            mutex_unlock(&dev->lock);
            break;
        default:
            ret = -EINVAL;
            break;
        }
        break;
    default:
        ret = -EINVAL;
        break;
    }

    return ret;
}

static int mpu6050_write_raw_get_fmt(struct iio_dev *indio_dev,
                    struct iio_chan_spec const *chan,
                    long mask){
    switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:		/* 用户空间写的陀螺仪分辨率数据要乘以1000000 */
			return IIO_VAL_INT_PLUS_MICRO;
		default:				/* 用户空间写的加速度计分辨率数据要乘以1000000000 */
			return IIO_VAL_INT_PLUS_NANO;
		}
	default:
		return IIO_VAL_INT_PLUS_MICRO;
	}
	return -EINVAL;
}

/* iio_info */
static const struct iio_info mpu6050_info = {
    .driver_module      = THIS_MODULE,
    .read_raw           = mpu6050_read_raw,
    .write_raw          = mpu6050_write_raw,
    .write_raw_get_fmt  = mpu6050_write_raw_get_fmt,
};

static int mpu6050_probe(struct i2c_client *client,const struct i2c_device_id *id){
    int ret = 0;
    struct mpu6050_dev *dev;
    struct iio_dev *indio_dev;
    printk("mpu6050_probe!\r\n");
    
    /* 1.申请iio_dev,和MPU6050_dev */
    indio_dev = devm_iio_device_alloc(&client->dev,sizeof(*dev));
    if(!indio_dev){
        ret = -ENOMEM;
        goto fail_iio_dev;
    }

    /* 2.获取MPU6050结构体地址 */
    dev = iio_priv(indio_dev);
    dev->client = client;

    /* 保存mpu6050结构体 */
    i2c_set_clientdata(client,indio_dev);

    /* 初始化互斥锁 */
    mutex_init(&dev->lock);

    /* 初始化iio_dev */
    indio_dev->dev.parent = &client->dev;
	indio_dev->channels = mpu6050_channels;
	indio_dev->num_channels = ARRAY_SIZE(mpu6050_channels);
	indio_dev->name = MPU6050_NAME;	
	indio_dev->modes = INDIO_DIRECT_MODE;	/* 直接模式，提供sysfs接口 */
	indio_dev->info = &mpu6050_info;
	// indio_dev->available_scan_masks = ;

    /* 将iio_dev注册到内核 */
    iio_device_register(indio_dev);
    if (ret < 0) {
		dev_err(&client->dev, "iio_device_register failed\n");
		goto fail_iio_register;
	}

    /* regmap申请和初始化 */
    dev->regmap_config.reg_bits = 8;
    dev->regmap_config.val_bits = 8;
    dev->regmap = regmap_init_i2c(client,&dev->regmap_config);
    if (IS_ERR(dev->regmap)) {
		ret = PTR_ERR(dev->regmap);
		goto fail_regmap_init;
	}

    MPU6050_Init(dev);

    return 0;
fail_regmap_init:
    iio_device_unregister(indio_dev);
fail_iio_register:
fail_iio_dev:
    return ret;
}

static int mpu6050_remove(struct i2c_client *client){
    int ret = 0;

    struct iio_dev *indio_dev = i2c_get_clientdata(client);
    struct mpu6050_dev *dev;

    dev = iio_priv(indio_dev);

    /* 1.注销iio_dev */
    iio_device_unregister(indio_dev);

    /* 2.删除regmap */
    regmap_exit(dev->regmap);
    return ret;
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

