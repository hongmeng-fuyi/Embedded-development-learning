
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
		printf("can not open file /dev/fd\n");
		return -1;
	}

    while(1){
		sleep(1);
		if(read(fd,&ns,4)==4)
		{
			//1us微妙=0.001ms毫秒 1s = 1000ms
			printf("get ns:%d\n",ns); 
			printf("get distance:%d mm\n",ns*340/(2*1000000)); 
		}
		else{
			printf("get distance failure!\n");
		}
    }


	
	close(fd);
	
	return 0;
}


