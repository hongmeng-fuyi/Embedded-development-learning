
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>


int main(int argc, char **argv)
{
	int fd;
	char buf[1024];
	int len;
	

	/* 2. 打开文件 */
	fd = open("/dev/", O_RDWR);
	if (fd == -1)
	{
		printf("can not open file /dev/hello\n");
		return -1;
	}

    while(1){
        len = read(fd, buf, 1024);		
        buf[1023] = '\0';
        printf("APP read : %s\n", buf);
    }


	
	close(fd);
	
	return 0;
}


