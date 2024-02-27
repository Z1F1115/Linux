#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>

/**
 * argc
 * argv
 * ./imx6uirqAPP <filename> 
 * ./imx6uirqAPP /dev/imx6uirq
 * */
int main(int argc,char **argv){
    int fd,ret;
    char *filename;
    unsigned char data;
    
    

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
        ret = read(fd ,&data,sizeof(data));
        if(ret < 0){

        }else {
            if(data){
                printf("key value = %#x\r\n",data);
            }
        }
    }
    

    close(fd);

    return 0;
}


