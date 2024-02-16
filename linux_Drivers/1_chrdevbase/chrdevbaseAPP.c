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
 * ./chrdevbaseAPP <filename> <1:2> 1表示读 2表示写
 * */
int main(int argc,char **argv){
    int ret = 0;
    int fd = 0;
    char *filename;
    char readbuf[100];
    char writebuf[100];
    static char usrdata[] = {"usr data!"};

    if(argc != 3){
        perror("Error usage!");
        exit(1);
    }

    filename = argv[1];

    fd = open(filename,O_RDWR);
    if(-1 == fd){
        perror("open error");
        exit(-1);
    }

    if(1 == atoi(argv[2])){
        ret = read(fd, readbuf, 50);
        if(-1 == ret){
            perror("read error");
            exit(-1);
        }
        printf("APP read data : %s \r\n",readbuf);
    }
    
    if(2 == atoi(argv[2])){
        memcpy(writebuf,usrdata,sizeof(usrdata));
        ret = write(fd,writebuf,50);
        if(-1 == ret){
            perror("write error");
            exit(-1);
        }

    }
    

    ret = close(fd);
    if(-1 == ret){
        perror("close error");
        exit(-1);
    }

    return 0;
}