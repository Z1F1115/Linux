#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>

/**
 * argc
 * argv
 * ./beepAPP <filename> <0:1> 0表示关闭蜂鸣器 1表示打开蜂鸣器
 * ./beepAPP /dev/miscbeep 0  关闭蜂鸣器
 * ./beepAPP /dev/miscbeep 1  打开蜂鸣器
 * */

int main(int argc,char **argv){
    int fd,retvalue;
    char *filename;
    unsigned char databuf[1];

    if(argc != 3){
        printf("Error Usage!\r\n");
        return -1;
    }

    filename = argv[1];

    fd = open(filename,O_RDWR);
    if(fd < 0){
        printf("file %s open failed!\r\n",filename);
        return -1;
    }

    databuf[0] = atoi(argv[2]);
    retvalue = write(fd,databuf,sizeof(databuf));
    if(retvalue < 0){
        printf("LED Control Failed!\r\n");
        close(fd);
        return -1;
    }

    close(fd);

    return 0;
}