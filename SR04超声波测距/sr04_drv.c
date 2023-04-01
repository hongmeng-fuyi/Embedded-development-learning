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
static struct class *sr04_class;
static struct gpio_desc *sr04_trig;
static struct gpio_desc *sr04_echo;
static int irq;
static int sr04_data_ns = 0;  
static wait_queue_head_t sr04_wq;

/* 实现对应的open/read/write等函数，填入file_operations结构体                   */
static ssize_t sr04_drv_read (struct file *file, char __user *buf, size_t size, loff_t *offset)
{
    /*
    手册解释测距操作：
        触发：向trig引脚发出一个大约10us的高电平，触发模块发射超声波
        回响:模块收到反射回来的超声波后，echo引脚输出一个高电平
        计算echo引脚到高电平的时间T 即Distance=340*T/2
    
    */
	int us = 0;
	unsigned long flags;
	int timeout_us = 1000000;
	


	/* 发送10us高电平    , 测量距离 2cm-450cm */
	gpiod_set_value(sr04_trig, 1);
	udelay(15);
	gpiod_set_value(sr04_trig, 0);
	
    /*等待数据*/
    wait_event_interruptible(sr04_wq,sr04_data_ns);

	copy_to_user(buf, &sr04_data_ns, 4);
    sr04_data_ns=0;
	return 4;
}


static unsigned int sr04_drv_poll(struct file *fp, poll_table * wait)
{
//	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
//	poll_wait(fp, &sr04_wait, wait);
	return 0;
}

/* 定义自己的file_operations结构体                                              */
static struct file_operations sr04_fops = {
	.owner	 = THIS_MODULE,
	.read    = sr04_drv_read,
	.poll    = sr04_drv_poll,
};

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
	wake_up(&sr04_wq);
	
	return IRQ_HANDLED; // IRQ_WAKE_THREAD;
}



static int sr04_probe(struct platform_device *pdev)
{
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
	/* 1. 获得硬件信息  trig是输出引脚，echo是输入引脚*/
	sr04_trig = gpiod_get(&pdev->dev, "trig", GPIOD_OUT_LOW);
	sr04_echo = gpiod_get(&pdev->dev, "echo", GPIOD_IN);

    //gpiod_to_irq使用该函数获取中断号
	irq = gpiod_to_irq(sr04_echo);
    //echo引脚设置为双边沿触发，上升触发中断 下降触发中断
    /*中断服务程序 sr04_isr**/
	request_irq(irq, sr04_isr, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, "sr04", NULL);

	/* 2. device_create */
	device_create(sr04_class, NULL, MKDEV(major, 0), NULL, "sr04");

	return 0;
}

static int sr04_remove(struct platform_device *pdev)
{
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
	device_destroy(sr04_class, MKDEV(major, 0));
//	free_irq(irq, NULL);
	gpiod_put(sr04_trig);
	gpiod_put(sr04_echo);
	return 0;
}

static const struct of_device_id ask100_sr04[] = {
    { .compatible = "100ask,sr04" },
    { },
};

/* 1. 定义platform_driver */
static struct platform_driver sr04s_driver = {
    .probe      = sr04_probe,
    .remove     = sr04_remove,
    .driver     = {
        .name   = "100ask_sr04",
        .of_match_table = ask100_sr04,
    },
};

static int __init sr04_init(void)
{
    int err;
    
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);

	/* 注册file_operations 	*/
	major = register_chrdev(0, "sr04", &sr04_fops);  

	sr04_class = class_create(THIS_MODULE, "sr04_class");
	if (IS_ERR(sr04_class)) {
		printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
		unregister_chrdev(major, "sr04");
		return PTR_ERR(sr04_class);
	}

	init_waitqueue_head(&sr04_wq);

	
    err = platform_driver_register(&sr04s_driver); 
	
	return err;
}


static void __exit sr04_exit(void)
{
	printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);

    platform_driver_unregister(&sr04s_driver);
	class_destroy(sr04_class);
	unregister_chrdev(major, "sr04");
}

module_init(sr04_init);
module_exit(sr04_exit);

MODULE_LICENSE("GPL");
