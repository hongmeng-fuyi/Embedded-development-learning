#include <linux/module.h>
#include <linux/poll.h>

#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/kmod.h>
#include <linux/gfp.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <asm/current.h>
#include <linux/delay.h>




static int major;
static struct class *dht11_class;
static int irq;
static wait_queue_head_t dht11_wq;
static struct gpio_desc *dht11_data_pin;
static u64 dht11_edge_time[82];//读5个字节也就是40个bit 每个bit数需要两个中断2，所以最少80，但第一个bit需要3个中断
static int dht11_edge_cnt;


static void dht11_start(void)
{
	u64 pre, last;
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
	//开始信号 = 一个低脉冲 + 一个高脉冲。低脉冲至少持续18ms，高脉冲持续20-40us
	/*首先将引脚配置为输出引脚，然后发开始信号*/
	gpiod_direction_output(dht11_data_pin,GPIOD_OUT_HIGH);
	udelay(2);

	pre = ktime_get_boottime_ns();
	gpiod_set_value(dht11_data_pin,0);
	mdelay(20);
	last = ktime_get_boottime_ns();
	printk("delay:%lld\n",last-pre);
	gpiod_set_value(dht11_data_pin,1);
	pre = ktime_get_boottime_ns();
	udelay(40);
	last = ktime_get_boottime_ns();
	printk("delay:%lld\n",last-pre);
	gpiod_direction_input(dht11_data_pin);
	udelay(2);
}


static int dht11_wait_for_read(void)
{
	int timeout_us=20000;
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
	//响应信号 = 一个低脉冲 + 一个高脉冲。低脉冲持续80us，高脉冲持续80us。
	while(gpiod_get_value(dht11_data_pin) && --timeout_us)
	{
		udelay(1);
	}
	if(!timeout_us)
	{
		printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
		return -1;
	}
	/*低电平*/
	timeout_us=200;
	while(!gpiod_get_value(dht11_data_pin) && --timeout_us)
	{
		udelay(1);
	}
	if(!timeout_us)
	{
		printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
		return -1;
	}
	/*现在是高电平*/
	//等待低电平
	timeout_us=200;
	while(gpiod_get_value(dht11_data_pin) && --timeout_us)
	{
		udelay(1);
	}
	if(!timeout_us)
	{
		printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
		return -1;
	}
	return 0;
	
}
//读取一个字节的数据
static int dht11_read_btye(unsigned char *buf)
{
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
	//
	int i;
	unsigned char data =0;
	int us=0;
	int timeout_us=200;
	for(i=0;i<8;i++)
	{
		while(!gpiod_get_value(dht11_data_pin) && --timeout_us)
		{
			udelay(1);
		}
		if(!timeout_us)
		{
			return -1;
		}
		timeout_us=200;
		us=0;
		//累加高电平时间
		while(gpiod_get_value(dht11_data_pin) && --timeout_us)
		{
			udelay(1);
			us++;
			if(us>40)
			{
				/*get bit 1*/
				data=(data<<1)|1;
			}
			else{
				/*get bit 0*/
				data=(data<<1)|0;
			}
		}
		if(!timeout_us)
		{
			return -1;
		}	

	}
	*buf=data;
}
/*中断服务程序*/
static irqreturn_t sr04_isr(int irq, void *dev_id)
{
	int val = gpiod_get_value(sr04_echo);
	/* 1. 记录数据 */
    if(val)
    {
        //echo引脚上升沿记录时间
        sr04_data_ns=ktime_get_ns();
    }
    else{//下降沿
        sr04_data_ns=ktime_get_ns()-sr04_data_ns;
    }
	//printk("%s %s %d, val = 0x%x\n", __FILE__, __FUNCTION__, __LINE__, sr04_data_us);

	/* 2. 唤醒APP:去同一个链表把APP唤醒 */
	wake_up(&dht11_wq);
	
	return IRQ_HANDLED; // IRQ_WAKE_THREAD;
}

int dht11_parse_data(char *data)
{

	int i=0,j=0,m=0;
	for(i=0;i<5;i++)
	{
		for(j=0;i<8;j++)
		{
			data[i]<<=1;
			if((dht11_edge_time[m+1]-dht11_edge_time[m])>=40000)
				data[i]|=1;
			m+=2;
		}
	}
	if(data[4]!=data[0]+data[1]+data[2]+data[3])
	{
		return -1;
	}
	else
		return 0;


}
/*中断服务程序*/
static irqreturn_t dht11_isr(int irq, void *dev_id){

	//记录时间
	dht11_edge_time[dht11_edge_cnt++]=ktime_get_boot_ns();
	if(dht11_edge_cnt >= 80)
	{
		dht11_data=1;
		/* 2. 唤醒APP:去同一个链表把APP唤醒 */
		wake_up(&dht11_wq);

	}


}
/* 实现对应的open/read/write等函数，填入file_operations结构体                   */
static ssize_t dht11_drv_read (struct file *file, char __user *buf, size_t size, loff_t *offset)
{
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
	unsigned long flags;
	int i=0;
	unsigned char data[5];

	/*发送启动信号*/

	dht11_start();

	//中断设置为上升触发中断 下降触发中断
	request_irq(irq, dht11_isr, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, "dht11", NULL);


	if(!wait_event_timeout(dht11_wq,dht11_data,HZ))
	{
		//超时退出
		free_irq(irq,NULL);//注意必须先释放中断然后才能复位
		dht11_start();
		
		return -ETIMEDOUT;
	}
	else{
		if(!dht11_parse_data(data))
		{
			/*copy_to_user*/
			copy_to_user(buf,&data,4);
		}
		else{

			return -EAGAIN;
		}
		free_irq(irq, NULL);
		dht11_start();
		
	}


	return 5;
}


static unsigned int dht11_drv_poll(struct file *fp, poll_table * wait)
{
//	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
//	poll_wait(fp, &dht11_wait, wait);
	return 0;
}

/* 定义自己的file_operations结构体                                              */
static struct file_operations dht11_fops = {
	.owner	 = THIS_MODULE,
	.read    = dht11_drv_read,
	.poll    = dht11_drv_poll,
};





static int dht11_probe(struct platform_device *pdev)
{
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
	/* 1. 获得硬件信息  trig是输出引脚，echo是输入引脚*/
	dht11_data_pin = gpiod_get(&pdev->dev, NULL, GPIOD_OUT_LOW);
	if(IS_ERR(dht11_data_pin))
	{
		printk("get dht11_data_pin failure\n");
	}
	 //gpiod_to_irq使用该函数获取中断号
	irq = gpiod_to_irq(dht11_data_pin);


	/* 2. device_create */
	device_create(dht11_class, NULL, MKDEV(major, 0), NULL, "dht11");

	return 0;
}

static int dht11_remove(struct platform_device *pdev)
{
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
	device_destroy(dht11_class, MKDEV(major, 0));
//	free_irq(irq, NULL);
	gpiod_put(dht11_data_pin);
	return 0;
}

static const struct of_device_id ask100_dht11[] = {
    { .compatible = "100ask,dht11" },
    { },
};

/* 1. 定义platform_driver */
static struct platform_driver dht11s_driver = {
    .probe      = dht11_probe,
    .remove     = dht11_remove,
    .driver     = {
        .name   = "100ask_dht11",
        .of_match_table = ask100_dht11,
    },
};

static int __init dht11_init(void)
{
    int err;
    
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);

	/* 注册file_operations 	*/
	major = register_chrdev(0, "dht11", &dht11_fops);  

	dht11_class = class_create(THIS_MODULE, "dht11_class");
	if (IS_ERR(dht11_class)) {
		printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
		unregister_chrdev(major, "dht11");
		return PTR_ERR(dht11_class);
	}

	//init_waitqueue_head(&dht11_wq);

	
    err = platform_driver_register(&dht11s_driver); 
	
	return err;
}


static void __exit dht11_exit(void)
{
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);

    platform_driver_unregister(&dht11s_driver);
	class_destroy(dht11_class);
	unregister_chrdev(major, "dht11");
}

module_init(dht11_init);
module_exit(dht11_exit);

MODULE_LICENSE("GPL");
