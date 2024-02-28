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
 * ./keyinputAPP <filename> 
 * ./keyinputAPP /dev/input/event1
 * */

/* input_event结构体 */
static struct input_event inputevent;

int main(int argc,char **argv){
    int fd,err;
    char *filename; 

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
        err = read(fd,&inputevent,sizeof(inputevent));
        if(err > 0){     /* 数据读取成功 */
            switch(inputevent.type){
                case EV_KEY:
                    if(inputevent.code < BTN_MISC){     /* KEY */
                        printf("key %d %s\r\n",inputevent.code,inputevent.value?"press":"release");
                    }else{
                        printf("button %d %s\r\n",inputevent.code,inputevent.value?"press":"release");
                    }
                    break;
                case EV_SYN:
                
                    break;
                case EV_REL:
                    break;
                case EV_ABS:
                    break;
            }
        } else{     /*  */
            printf("数据读取失败！\r\n");
        }
    }
    

    close(fd);

    return 0;
}


