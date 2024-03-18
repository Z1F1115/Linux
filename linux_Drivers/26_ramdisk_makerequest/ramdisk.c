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
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <linux/genhd.h>
#include <linux/blkdev.h>
#include <linux/hdreg.h>

#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>



/* 定义磁盘大小，内存模拟 */
#define RAMDISK_SIZE (2 * 1024 * 1024)		/* 大小2MB */
#define RAMDISK_NAME	"ramdisk"			/* 名字 */
#define RAMDISK_MINOR	3					/* 表示3个分区 */

/* ramdisk设备结构体 */
struct ramdisk_dev{
	int major;		/* 主设备号 */
	unsigned char *ramdiskbuf;	/* ramdisk的内存空间，模拟磁盘的空间 */
	struct gendisk *gendisk;
	struct request_queue *queue;
	spinlock_t lock;	/* 自旋锁 */

};

struct ramdisk_dev ramdisk;

#if 0
/* 具体的数据处理过程 */
static void ramdisk_transfer(struct request *req){
	/* 数据传输三要素：源，目的，长度。内存地址，块设备地址，长度 */
	unsigned long start = blk_rq_pos(req) << 9;
	unsigned long len = blk_rq_cur_bytes(req);

	/* 获取bio里面到缓冲区：
	 * 如果是读：从磁盘里面读写到到数据保存在次缓冲区里面
	 * 如果是写：此缓冲区保存着要写入到磁盘里面的数据
	 */
	void *buffer = bio_data(req->bio);

	if(rq_data_dir(req) == READ){		/* 读操作 */
		memcpy(buffer, ramdisk.ramdiskbuf + start,len);
	}else{								/* 写操作 */
		memcpy(ramdisk.ramdiskbuf + start, buffer,len);
	}
}
#endif

/* 制造请求函数 */
static void ramdisk_make_request(struct request_queue *queue, struct bio *bio){
	int offset;
	struct bio_vec bvec; 
	struct bvec_iter iter;
	unsigned long len = 0;

	offset = bio->bi_iter.bi_sector << 9;		/* 要操作的磁盘起始扇区偏移,改为字节地址 */

	/* 循环处理每个断 */
	bio_for_each_segment(bvec, bio, iter){
		/* 获取bio里面到缓冲区：
		* 如果是读：从磁盘里面读写到到数据保存在次缓冲区里面
		* 如果是写：此缓冲区保存着要写入到磁盘里面的数据
		*/
		char *ptr = page_address(bvec.bv_page) + bvec.bv_offset;
		len = bvec.bv_len;	/* 长度 */

		if(bio_data_dir(bio) == READ){		/* 读操作 */
		memcpy(ptr, ramdisk.ramdiskbuf + offset,len);
		}else{								/* 写操作 */
			memcpy(ramdisk.ramdiskbuf + offset, ptr,len);
		}
		offset += len;

	}

	set_bit(BIO_UPTODATE, &bio->bi_flags);
	bio_endio(bio,0);

}

static int ramdisk_open(struct block_device *bdev,fmode_t mode){
	printk("ramdisk_open\r\n");

	return 0;
}

static void ramdisk_release(struct gendisk *disk,fmode_t mode){
	printk("ramdisk_release\r\n");

}

static int ramdisk_getgeo(struct block_device *bdev,struct hd_geometry *geo){
	printk("ramdisk_getgeo\r\n");

	/* 磁盘信息 */
	geo->heads = 2;	/* 磁头 */
	geo->cylinders = 32; /* 柱面 */
	geo->sectors = RAMDISK_SIZE / (2 * 32 * 512);	/* 一个磁道里面的扇区数量 */

	return 0;
}

/* 块设备操作集 */
static const struct block_device_operations ramdisk_fops = {
	.owner		= THIS_MODULE,
	.open		= ramdisk_open,
	.release	= ramdisk_release,
	.getgeo		= ramdisk_getgeo,
};	

/* 驱动入口函数 */
static int __init ramdisk_init(void){
	int ret = 0;

	printk("ramdisk_init\r\n");

	/* 1、申请内存 */
	ramdisk.ramdiskbuf = kzalloc(RAMDISK_SIZE,GFP_KERNEL);
	if(ramdisk.ramdiskbuf == NULL){
		ret = -EINVAL;
		goto ramalloc_fail;
	}

	/* 2、注册块设备 */
	ramdisk.major = register_blkdev(0,RAMDISK_NAME);
	if(ramdisk.major < 0){
		ret = -EINVAL;
		goto ramdisk_register_blkdev_fail;
	}
	printk("ramdisk major = %d\r\n",ramdisk.major);

	/* 3、申请gendisk磁盘 */
	ramdisk.gendisk = alloc_disk(RAMDISK_MINOR);
	if(!ramdisk.gendisk){
		ret = -EINVAL;
		goto gendisk_alloc_fail;
	}

	/* 4、初始化自旋锁 */
	spin_lock_init(&ramdisk.lock);

	/* 5、申请请求队列 */
	ramdisk.queue = blk_alloc_queue(GFP_KERNEL);
	if(!ramdisk.queue){
		ret = -EINVAL;
		goto blk_queue_fail;
	}

	/* 绑定“制造请求”函数 */
	blk_queue_make_request(ramdisk.queue,ramdisk_make_request);

	/* 6、初始化gendisk */
	ramdisk.gendisk->major = ramdisk.major;		/* 主设备号 */
	ramdisk.gendisk->first_minor = 0;
	ramdisk.gendisk->fops = &ramdisk_fops;
	ramdisk.gendisk->private_data = &ramdisk;
	ramdisk.gendisk->queue = ramdisk.queue;
	sprintf(ramdisk.gendisk->disk_name,RAMDISK_NAME);
	set_capacity(ramdisk.gendisk,RAMDISK_SIZE / 512);		/* 设置gendisk容量，单位扇区 */
	add_disk(ramdisk.gendisk);


	return 0;
blk_queue_fail:
	put_disk(ramdisk.gendisk);
gendisk_alloc_fail:
	unregister_blkdev(ramdisk.major,RAMDISK_NAME);
ramdisk_register_blkdev_fail:
	kfree(ramdisk.ramdiskbuf);
ramalloc_fail:
	return ret;
}

/* 驱动出口函数 */
static void __exit ramdisk_exit(void){
	printk("ramdisk_exit\r\n");
	del_gendisk(ramdisk.gendisk);
	put_disk(ramdisk.gendisk);

	blk_cleanup_queue(ramdisk.queue);

	unregister_blkdev(ramdisk.major,RAMDISK_NAME);
	kfree(ramdisk.ramdiskbuf);
}

module_init(ramdisk_init);
module_exit(ramdisk_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhongjunfeng");

