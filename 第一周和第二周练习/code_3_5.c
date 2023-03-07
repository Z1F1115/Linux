#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

//int argc,char *argv[]
int main(){
	
	int count = 0;
	int len = 0;
	char pathname1[1024] = {0};
	char pathname2[1024] = {0};
	char read_buffer[1024] = {0};
	
	printf("请从键盘输入被拷贝文件的路径：\n");
	scanf("%s",pathname1);
	int fd1 = open(pathname1,O_RDWR,0777);
	if (-1 == fd1){
		printf("文件打开失败\n");
		return -1;
	}
	
	printf("请从键盘输入拷贝到文件的路径：\n");
	scanf("%s",pathname2);
	int fd2 = open(pathname2,O_RDWR|O_CREAT,0777);
	if (-1 == fd2){
		printf("文件打开失败\n");
		return -1;
	}
	
	ssize_t read_size,write_size;
	while (1){
		read_size = read(fd1,read_buffer,1024);
		if (read_size < 0){
			printf("读取文件错误");
			return -1;
		}
		if (0 == read_size) 
			break;
		else count++;
		
		len = read_size;
		printf("%d\n",len);
		write_size = write(fd2, read_buffer, len);
	}
	printf("%d\n",count);
	
	close(fd1);
	close(fd2);
	return 0;
}