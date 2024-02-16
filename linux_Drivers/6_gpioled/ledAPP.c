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
 * ./ledAPP <filename> <0:1> 0表示关灯 1表示开灯
 * ./ledAPP /dev/dtsled 0  关灯
 * ./ledAPP /dev/dtsled 1  开灯
 * */

#define LEDOFF  0
#define LEDON   1

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


