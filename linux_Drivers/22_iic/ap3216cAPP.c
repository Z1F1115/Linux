#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/input.h>

/**
 * argc
 * argv
 * ./ap3216cAPP <filename> 
 * ./ap3216cAPP /dev/ap3216c
 * */

int main(int argc,char **argv){
    int fd,err;
    char *filename; 
    unsigned short data[3];
    unsigned short ir,ps,als;

    short mpu6050_data[7];
    short acc_x,acc_y,acc_z,temp,gyr_x,gyr_y,gyr_z;

    if(argc != 2){
        printf("Error Usage!\r\n");
        return -1;
    }

    filename = argv[1];

    fd = open(filename,O_RDWR);
    if(fd < 0){
        printf("file %s open key!\r\n",filename);
        return -1;
    }

    while(1){
        
        err = read(fd, data, sizeof(data));
        if(err == 0){
            ir = data[0];
            als = data[1];
            ps = data[2];
            printf("AP3216C ir = %d ,als = %d ,ps = %d\r\n",ir,als,ps);
        }
        
#if 0
        err = read(fd, mpu6050_data, sizeof(mpu6050_data));
        if(err == 0){
            gyr_x = mpu6050_data[0];
            gyr_y = mpu6050_data[1];
            gyr_z = mpu6050_data[2];
            acc_x = mpu6050_data[3];
            acc_y = mpu6050_data[4];
            acc_z = mpu6050_data[5];
            temp = mpu6050_data[6];
            printf("ACC_X = %d , ACC_Y = %d , ACC_Z = %d\r\n",acc_x,acc_y,acc_z);
            printf("GYR_X = %d , GYR_Y = %d , GYR_Z = %d\r\n",gyr_x,gyr_y,gyr_z);
            printf("TEMP = %d \r\n",temp);
        }
#endif
        usleep(200000);     /* 200ms */
    }

    close(fd);

    return 0;
}


