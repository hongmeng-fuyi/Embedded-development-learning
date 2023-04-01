
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>


int main(int argc, char **argv)
{
	int fd;
	unsigned char data[4];
	int ns;
	
	if(argc!=2)
	{
		printf("please usage dev fd\n");
		return -1;
	}

	/* 2. 打开文件 */
	fd = open(argv[1], O_RDWR);
	if (fd == -1)
	{
		printf("can not open file %s\n",argv[1]);
		return -1;
	}

    while(1){
		sleep(1);
		if(read(fd,data,4)==4)
		{
			printf("get humidity:%d\n",data[0]);
			printf("get temprature:%d\n",data[2]);
		}
		else{
			printf("get humidity temprature failure!\n");
			sleep(1);
		}
    }


	
	close(fd);
	
	return 0;
}


