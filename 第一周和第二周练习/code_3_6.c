#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

int main(){
	char pathname[1024] = {0};
	char new_pathname[1024] = {0};
	char content[1024] = {0};
	char read_buffer[1024] = {0};
	int n = 0;
	int len = 0;
	
	printf("请从键盘输入要被插入的文件路径：\n");
	scanf("%s",pathname);
	
	int fd = open(pathname,O_RDWR);
	if(-1 == fd){
		printf("打开文件失败\n");
		return -1;
	}
	
	printf("请从键盘输入要被插入的文件路径：\n");
	scanf("%s",new_pathname);
	
	int new_fd = open(new_pathname,O_RDWR | O_CREAT,0777);;
	if (-1 == new_fd){
		printf("打开文件失败\n");
		return -1;
	}
	
	printf("请输入在第几个字符插入:>");
	scanf("%d",&n);
	
	lseek(fd,n,SEEK_SET);
	
	ssize_t read_size;
	while (1){
		read_size = read(fd,read_buffer,sizeof(read_buffer));
		if (0 == read_size) break;
		
		write(new_fd,read_buffer,strlen(read_buffer));
	}
	
	lseek(fd,n,SEEK_SET);
	
	printf("请从键盘输入要插入内容:>\n");
	scanf("%s",content);
		
	write(fd,content,strlen(content));
	
	lseek(new_fd,0,SEEK_SET);
	
	while (1){
		read_size = read(new_fd,read_buffer,sizeof(read_buffer));
		if (0 == read_size) break;
		
		write(fd,read_buffer,strlen(read_buffer));
	}
	
	remove(new_pathname);
	
	close(fd);
	close(new_fd);
	return 0;
}