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


static int irq;
static int major = 0;
static struct class * sr501_class;
static struct gpio_desc *sr501_gpio;
static wait_queue_head_t sr501_wq;/*等待队列,用来存放调用我的app，注意在入口函数先初始化*/
static int has_data;

static const struct of_device_id stm157_sr501_table[]={
    /* data */
    { .compatible =  "100ask,stm157_sr501" },
    { },
};

/*使用中断方式*/
static ssize_t sr501_drv_read(struct file *file, char __user *buf, size_t size, loff_t *offset)
{
#if 0
    printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
    int val=0;
    int len = (size<4)?size:4
    val = gpiod_get_value(sr501_gpio);
    copy_to_user(buf,&val,len)
    return len;
#else
        int val=0;
        int len = (size<4)?size:4
        
        /*有数据就copy_to_user 没有数据就休眠不读取*/
        //等待事件，等待有数据产生的事件.has_data!=0的时候copy_to_user,否则App在等待队列中等待
        if(wait_even_interruptible(sr501_wq,has_data)==0){//has_data 是有中断的时候有数据？？
        /*也就是中断触发的时候，我们获取数据，即在sr501_isr函数取数据*/
            copy_to_user(buf,&has_data,len);
            has_data=0;
            return len;
        
        }
        else{
            return -EAGAIN;
        }

#endif
}
/*使用内核线程*/
static int sr501_detect(void *arg)
{
	int val;
	int pre = -1;
	while (1)
	{
		val = gpiod_get_value(sr501_gpio);

		if (pre != val)
		{
			printk("%s %s %d, val = 0x%x\n", __FILE__, __FUNCTION__, __LINE__, val); 	
			sr501_data = 0x80|val;
			wake_up(&sr501_wq);
			pre = val;
		}
		
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(HZ);

		if (kthread_should_stop()) {
			set_current_state(TASK_RUNNING);
			break;
		}
		
	}
	return 0;

}


/*中断处理函数*/
static irqreturn_t sr501_isr(int irq, void *dev_id)
{
	/* 1. 记录数据 */
	has_data = 1;

	/* 2. 唤醒APP:我只是个驱动由使用我的应用程序app唤醒 */
	wake_up(&sr501_wq);
	
	return IRQ_HANDLED;
}


static int sr501_thread_func(void *data){
    int cnt=0;
    while(!kthread_should_stop()){
        printk("%s %s %d, cnt =%d\n", __FILE__, __FUNCTION__, __LINE__, cnt++);
        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(5*HZ);//休眠5秒
    }
}

static unsigned int sr501_drv_poll(struct file *fp, poll_table * wait)
{
    return 0;
};

static struct file_operations sr501_opt=
{
    /* data */
    .owner = THIS_MODULE,
    .read = sr501_drv_read,
    .poll = sr501_drv_poll,
};

/*从设备树获取硬件信息*/
static int sr501_drv_probe(struct platform_device *pdev)
{
    printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
    /*获取GPIO信息*/
    sr501_gpio = gpiod_get(&pdev->dev,NULL,0);
    /*设置gpio的模式*/
    gpiod_direction_input(sr501_gpio);

    /*获取该引脚对应的 IRQ number即中断号*/
    irq = gpiod_to_irq(sr501_gpio);
    /*给中断注册一个中断处理函数*/
	request_irq(irq, sr501_isr, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, "sr501", NULL);

    /*不使用中断，使用内核线程，创建内核线程*/
    //kernel_thread创建成功后是未被激活的，不能工作，如果需要工作，则需要使用wake_up_process()函数来唤醒。线程一旦启动起来后，会一直运行，除非该线程主动调用do_exit函数，或者其他的进程调用kthread_stop函数，本线程可以使用kthread_should_stop()来获取它其他线程kthread_stop()信号，
    //kernel_thread(sr501_detect,NULL,0);//kernel_thread()函数是通过调用do_fork()**函数创建的线程
    sr501_kthread=kthread_run(sr501_thread_func,NULL,"sr501d");//创建并唤醒该线程,kthread_run()调用kthread_create()

	//把file_operations告诉内核，即register，分配一个major设备号
	major = register_chrdev(0, "sr501", &sr501_opt); 

    /*这个函数开始执行逻辑程序的流程，即主设备号这些 class_create + device_create*/
    //class_create + device_create结合使得自动创建设备节点
	sr501_class = class_create(THIS_MODULE, "sr501_class");
    
    device_create(sr501_class, NULL, MKDEV(major, 0), NULL, "sr501_drv"); 

    return 0;

};

static int sr501_drv_remove(struct platform_device *pdev)
{
    printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
	device_destroy(sr501_class, MKDEV(major, 0));
	class_destroy(sr501_class);
	unregister_chrdev(major, "sr501");
    free_irq(irq,NULL);
    gpiod_put(sr501_gpio);

    return 0;
};

/*1、定义一个platform_driver结构体进行注册*/
static struct platform_driver sr501_driver =
{
    /* data */
    .probe= sr501_drv_probe,
    .remove = sr501_drv_remove,
    .driver = {
        .name ="stm157_sr501",
        .of_match_table=stm157_sr501_table,
    },
};



/*2、驱动入口函数注册platform_driver*/
static int __init sr501_init(void)
{
    int err;
    printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
    /*初始化等待队列*/
    init_waitqueue_head(&sr501_wq);
    err = platform_driver_register(&sr501_driver);

    return err;

}


static void __exit sr501_exit(void)
{
    printk("%s %s line %d\n", __FILE__, __FUNCTION__, __LINE__);
    platform_driver_unregister(&sr501_driver);
    class_destroy(sr501_class);
	unregister_chrdev(major, "sr501");

    kthread_stop(sr501_kthread);
}

module_init(sr501_init);
module_exit(sr501_exit);

MODULE_LICENSE("GPL");