#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>


#define KEY0VALUE   0XF0
#define INVAKEY     0X00

/**
 * argc
 * argv
 * ./keyAPP <filename> 
 * ./keyAPP /dev/key 
 * */
int main(int argc,char **argv){
    int value;
    int fd,retvalue;
    char *filename;
    unsigned char databuf[1];

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
        read(fd,&value,sizeof(value));
        if(value == KEY0VALUE){
            printf("KEY0 Press,value = %d\r\n",value);
        }
    }
    

    close(fd);

    return 0;
}


