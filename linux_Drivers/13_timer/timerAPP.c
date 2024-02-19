#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>

#define CLOSE_CMD       _IO(0XEF,1)        //关闭命令
#define OPEN_CMD        _IO(0XEF,2)        //打开命令
#define SETPERIOD_CMD   _IOW(0XEF,3,int)   //设置周期

/**
 * argc
 * argv
 * ./timerAPP <filename> 
 * ./timerAPP /dev/timer
 * */
int main(int argc,char **argv){
    int fd,ret;
    char *filename;
    unsigned char databuf[1];
    unsigned int cmd;
    unsigned int arg;
    unsigned char str[100];

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

    /* 循环读取 */
    while (1)
    {
        printf("Input CMD:");
        ret = scanf("%d",&cmd);
        if (ret != 1){
            gets(str);      /* 防止卡死 */
        }

        if(cmd == 1){       /* 关闭 */
            ioctl(fd,CLOSE_CMD,&arg);
        }   
        else if(cmd == 2){  /* 打开 */
            ioctl(fd,OPEN_CMD,&arg);
        }
        else if(cmd == 3){  /* 设置周期 */
            printf("Input Timer period:");
            ret = scanf("%d",&arg);
            if(ret != 1){
                gets(str);
            }
            ioctl(fd,SETPERIOD_CMD,&arg);
        }
    }
    

    close(fd);

    return 0;
}


