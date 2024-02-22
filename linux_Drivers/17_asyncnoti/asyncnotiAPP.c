#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <fcntl.h>


/**
 * argc
 * argv
 * ./asyncnotiAPP <filename> 
 * ./asyncnotiAPP /dev/imx6uirq
 * */
int fd;

static void sigio_signal_func(int num){
    int err;
    unsigned int keyvalue = 0;

    err = read(fd, &keyvalue, sizeof(keyvalue));
    if(err < 0){

    }else {
        printf("sigio signal! key value = %d\r\n",keyvalue);
    }
}

int main(int argc,char **argv){
    int ret;
    char *filename;
    unsigned char data;
    int flags = 0;
    

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

    /* 设置信号处理函数 */
    signal(SIGIO,sigio_signal_func); 

    fcntl(fd, F_SETOWN, getpid());   /* 设置当前进程接收SIGIO信号 */
    flags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL , flags | FASYNC);     /* 异步控制 */

    while(1){
        sleep(2);
    }

    close(fd);

    return 0;
}


