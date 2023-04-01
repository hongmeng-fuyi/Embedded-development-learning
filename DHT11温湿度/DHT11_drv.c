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


void dht11_delay_us(int us)
{
	u64 pre, last;

	pre =ktime_to_us(ktime_get_boottime());
	while (1)
	{
		last = ktime_to_us(ktime_get_boottime());
		if (last - pre >= us)
			break;
	}

}

static void dht11_start(void)
{
	long long  pre, last;
	long long cnt=0;
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
	//开始信号 = 一个低脉冲 + 一个高脉冲。低脉冲至少持续18ms，高脉冲持续20-40us
	/*首先将引脚配置为输出引脚，然后发开始信号*/
	gpiod_direction_output(dht11_data_pin,GPIOD_OUT_HIGH);

	gpiod_set_value(dht11_data_pin,0);
	mdelay(20);
	
	gpiod_set_value(dht11_data_pin,1);
	dht11_delay_us(30);

	gpiod_direction_input(dht11_data_pin);

}


static int dht11_wait_for_read(void)
{
	int cnt=0;
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
	//响应信号 = 一个低脉冲 + 一个高脉冲。低脉冲持续80us，高脉冲持续80us。
	while(!gpiod_get_value(dht11_data_pin))
	{
		udelay(1);
		cnt++;
		if(cnt>=80)
		{
			printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
			cnt=0;
			while(gpiod_get_value(dht11_data_pin))
			{
				udelay(1);
				cnt++;
				if(cnt>=80)
				{
					printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
					cnt=0;
					while(!gpiod_get_value(dht11_data_pin))
					{
						udelay(1);
						cnt++;
						if(cnt>=80)
						{
							printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
							return 0;
						}
							
					}
				}
			}
		}
	}
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
	return -1;
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


/* 实现对应的open/read/write等函数，填入file_operations结构体                   */
static ssize_t dht11_drv_read (struct file *file, char __user *buf, size_t size, loff_t *offset)
{
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
	unsigned long flags;
	int i=0;
	unsigned char data[5];
	/*轮询计时方式，得关闭中断中*/
	local_irq_save(flags);//关闭中断
	/*发送启动信号*/

	dht11_start();
	/*等待就绪，注意设置超时时间，不能让我们一直等下去*/
	if(dht11_wait_for_read())
	{
		return -EAGAIN;
	}
	/*读数据
	:8bit湿度整数数据+8bit湿度小数数据
	+8bi温度整数数据+8bit温度小数数据
	+8bit校验和。
	*/
	for(i=0;i<5;i++)
	{
		if(dht11_read_btye(&data[i]))
		{
			local_irq_restore(flags);
			return -EAGAIN;
		}
	}
	local_irq_restore(flags);
	/*根据校验码验证数据*/
	if(data[4]!=(data[0]+data[1]+data[2]+data[3]))
		return -1;

	/*copy_to_user*/
	copy_to_user(buf,&data,4);
	gpiod_direction_output(dht11_data_pin,GPIOD_OUT_HIGH);
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
