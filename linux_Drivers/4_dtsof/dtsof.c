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

/* 模块入口 */
static int __init dtsof_init(void){
    int ret = 0;
    struct device_node *bl_nd = NULL;     /* 节点 */
    struct property *comppro = NULL;
    const char *str;
    u32 def_value = 0;
    u32 elemsize;
    u32 *brival;
    u8 i = 0;

    /* 1.找到backlight节点，路径是:/backlight */
    bl_nd = of_find_node_by_path("/backlight");
    if(bl_nd == NULL){
        ret = -EINVAL;
        goto faile_findnd;
    }

    /* 2.获取属性，字符 */
    comppro = of_find_property(bl_nd,"compatible",NULL);
    if(comppro == NULL){
        ret = -EINVAL;
        goto faile_finpro;
    }else{
        printk("compatible = %s\r\n",(char*)comppro->value);
    }

    ret = of_property_read_string(bl_nd,"status",&str);
    if(ret < 0){
        goto fail_rs;
    }else{
        printk("status = %s\r\n",str);
    }

    /* 3.获取数字属性值 */
    ret = of_property_read_u32(bl_nd,"default-brightness-level",&def_value);
    if(ret < 0){
        goto fail_read32;
    }else{
        printk("default-brightness-level = %d\r\n",def_value);
    }

    /* 4.获取数组类型的属性 */  
    elemsize = of_property_count_elems_of_size(bl_nd,"brightness-levels",sizeof(u32));
    if(elemsize < 0){
        ret = -EINVAL;
        goto fail_readele;
    }else{
        printk("brightness-levels elems size = %d\r\n",elemsize);
    }

    /* 申请内存 */
    brival = kmalloc(elemsize * sizeof(u32), GFP_KERNEL);
    if(!brival){
        ret = -EINVAL;
        goto fail_mem;
    }
    
    /* 获取数组 */
    ret = of_property_read_u32_array(bl_nd,"brightness-levels",brival,elemsize);
    if(ret < 0){
        goto fail_read32array;
    }else{
        for(i = 0; i < elemsize; i++){
            printk("brightness-levels[%d] = %d\r\n", i , *(brival + i));
        }
    }
    kfree(brival);

    return 0;

fail_read32array:  
    kfree(brival);
fail_mem:
fail_readele:
fail_read32:
fail_rs:
faile_finpro:
faile_findnd:
    return ret;
}

/* 模块出口 */  
static void __exit dtsof_exit(void){

}

/* 模块入口和出口 */
module_init(dtsof_init);
module_exit(dtsof_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhongjunfeng");

