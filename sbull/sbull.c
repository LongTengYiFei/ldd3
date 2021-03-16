/*
 * Sample disk driver, from the beginning.
 */
//毕设是linux 5.4
//会删除一些无效的不同版本的代码
#include <linux/version.h> 	/* LINUX_VERSION_CODE  */
#include <linux/blk-mq.h>	
/* https://olegkutkov.me/2020/02/10/linux-block-device-driver/
   https://prog.world/linux-kernel-5-0-we-write-simple-block-device-under-blk-mq/           
   blk-mq and kernels >= 5.0
*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/sched.h>
#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/timer.h>
#include <linux/types.h>	/* size_t */
#include <linux/fcntl.h>	/* O_ACCMODE */
#include <linux/hdreg.h>	/* HDIO_GETGEO */
#include <linux/kdev_t.h>
#include <linux/vmalloc.h>
#include <linux/genhd.h>
#include <linux/blkdev.h>
#include <linux/buffer_head.h>	/* invalidate_bdev */
#include <linux/bio.h>

MODULE_LICENSE("Dual BSD/GPL");

static int sbull_major = 0;
module_param(sbull_major, int, 0);
static int hardsect_size = 512;
module_param(hardsect_size, int, 0);
static int nsectors = 1024 * 20;	/* How big the drive is */
module_param(nsectors, int, 0);
static int ndevices = 1;
module_param(ndevices, int, 0);

/*
 * The different "request modes" we can use.
 */
enum {
	RM_SIMPLE  = 0,	/* The extra-simple request function */
	RM_FULL    = 1,	/* The full-blown version */
	RM_NOQUEUE = 2,	/* Use make_request */
	RM_MQ = 3,/* cyf two-level multi-queue mode*/
	RM_STACKBD = 4,
};
static int request_mode = RM_MQ;
module_param(request_mode, int, 0);

struct funny_mud_pee;

/*
 * Minor number and partition management.
 */
#define SBULL_MINORS	16
#define MINOR_SHIFT	4
#define DEVNUM(kdevnum)	(MINOR(kdev_t_to_nr(kdevnum)) >> MINOR_SHIFT

/*
 * We can tweak our hardware sector size, but the kernel talks to us
 * in terms of small sectors, always.
 */
#define KERNEL_SECTOR_SIZE	512

/*
 * After this much idle time, the driver will simulate a media change.
 */
//这个值之前设置了30，太短了，导致我每次fdisk ，mkfs之后就失效了，所以总是mount失败
//有的时候fdisk之后就失效了，连mkfs都不行。
#define INVALIDATE_DELAY	500*HZ


//我的一些ioctl命令的定义
#define SAYHELLO_CYFCMD  666666

/*
 * The internal representation of our device.
 */

struct sbull_hw_queue_private {
	unsigned int index;
	unsigned int queue_depth;
	struct sbull_dev *dev;
};
//sbull用内部的一个数据结构表示
struct sbull_dev {
        int size;                       /* Device size in sectors */
        u8 *data;                       /* The data array */
        short users;                    /* How many users */
        spinlock_t lock;                /* For mutual exclusion */
	spinlock_t hardware_queue_lock;
	//注意request_queue 这个结构体内部既包含了软队列指针也包含了硬队列指针
	//这是个大结构体
	//linux 5.4 源码
        struct request_queue *queue;    /* The device request queue */
	/*
	 *blk_mq_tag_set data struct is for kernel to manage the request-queue. Driver decides how many hw-queues will be created because driver know the HW. If HW supported two channel IO processing, driver would create two hw-queues. And driver stores the number of hw-queue and other information into blk_mq_tag_set object.
	 * */
	struct blk_mq_tag_set tag_set;	/* tag_set added */
	unsigned int queue_depth;	
        struct sbull_hw_queue_private *hw_queue_priv;
	
        struct gendisk *gd;             /* The gendisk structure */

	short media_change;             /* Flag a media change? */
        struct timer_list timer;        /* For simulated media changes */
};

static int nr_hw_queues = 1;
static int hw_queue_depth = 64;

static struct sbull_dev *Devices = NULL;


/*
 * Handle an I/O request.
 */
//这个是转发至dev里面的数组，也就是内存，vmalloc
static void sbull_transfer(struct sbull_dev *dev, unsigned long sector_index,
		unsigned long sect_count, char *buffer, int write)
{
	printk(KERN_ALERT"%s() begin.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	
	unsigned long dev_offset = sector_index*KERNEL_SECTOR_SIZE;
	unsigned long byte_count = sect_count*KERNEL_SECTOR_SIZE;

	if ((dev_offset + byte_count) > dev->size) {
		printk (KERN_NOTICE "Beyond-end write (%ld %ld)\n", dev_offset, byte_count);
		return;
	}
	if (write)
		memcpy(dev->data + dev_offset, buffer, byte_count);
	else
		memcpy(buffer, dev->data + dev_offset, byte_count);
	printk(KERN_ALERT"%s() over.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
}

/*
 * Transfer a single BIO.
 */
static int sbull_xfer_bio(struct sbull_dev *dev, struct bio *bio)
{
	printk(KERN_ALERT"%s() begin.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	struct bio_vec bvec;
	struct bvec_iter iter;
	sector_t sector = bio->bi_iter.bi_sector;

	/* Do each segment independently. */
	bio_for_each_segment(bvec, bio, iter) {
		char *buffer = kmap_atomic(bvec.bv_page) + bvec.bv_offset;
		sbull_transfer(dev, sector, (bio_cur_bytes(bio) / KERNEL_SECTOR_SIZE),
				buffer, bio_data_dir(bio) == WRITE);
		sector += (bio_cur_bytes(bio) / KERNEL_SECTOR_SIZE);
		kunmap_atomic(buffer);
	}
	printk(KERN_ALERT"%s() over.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	return 0; /* Always "succeed" */
}

/*
 * Transfer a full request.
 */
static int sbull_xfer_request(struct sbull_dev *dev, struct request *req)
{
	printk(KERN_ALERT"%s() begin.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	struct bio *bio;
	int sect_count = 0;
    
	__rq_for_each_bio(bio, req) {
		sbull_xfer_bio(dev, bio);
		//printk(KERN_ALERT"the bi_size is %d", bio->bi_iter.bi_size);
		sect_count += bio->bi_iter.bi_size/KERNEL_SECTOR_SIZE;
	}
	printk(KERN_ALERT"%s() over.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	return sect_count;
}

/*
 * The simple form of the request function.
 */
//static void sbull_simple_request(struct request_queue *q)
static blk_status_t sbull_simple_request(struct blk_mq_hw_ctx *hctx, const struct blk_mq_queue_data* bd)   /* For blk-mq */
{
	printk(KERN_ALERT"%s() begin.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	struct request *req = bd->rq;
	struct sbull_dev *dev = req->rq_disk->private_data;
        struct bio_vec bvec;
        struct req_iterator iter;
        sector_t pos_sector = blk_rq_pos(req);
	void	*buffer;
	blk_status_t  ret;

	blk_mq_start_request (req);

	if (blk_rq_is_passthrough(req)) {
		printk (KERN_NOTICE "Skip non-fs request\n");
                ret = BLK_STS_IOERR;  //-EIO
			goto done;
	}
	rq_for_each_segment(bvec, req, iter)
	{
		size_t num_sector = blk_rq_cur_sectors(req);
		printk (KERN_NOTICE "Req dev %u, request_data_direct %d, pos_sector %lld, num_sector %ld\n",
                        (unsigned)(dev - Devices), rq_data_dir(req),
                        pos_sector, num_sector);
		buffer = page_address(bvec.bv_page) + bvec.bv_offset;
		sbull_transfer(dev, pos_sector, num_sector,
				buffer, rq_data_dir(req) == WRITE);
		pos_sector += num_sector;
	}
	ret = BLK_STS_OK;
done:
	blk_mq_end_request (req, ret);
	printk(KERN_ALERT"%s() over.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	return ret;
}


/*
 * Smarter request function that "handles clustering".
 */
//static void sbull_full_request(struct request_queue *q)
static blk_status_t sbull_full_request(struct blk_mq_hw_ctx * hctx, const struct blk_mq_queue_data * bd)
{
	printk(KERN_ALERT"%s() begin.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	struct request *req = bd->rq;
	int sectors_xferred;
	struct sbull_dev *dev = req->q->queuedata;
	blk_status_t  ret;

	blk_mq_start_request (req);
		if (blk_rq_is_passthrough(req)) {
			printk (KERN_NOTICE "Skip non-fs request\n");
			ret = BLK_STS_IOERR; //-EIO;
			goto done;
		}
		sectors_xferred = sbull_xfer_request(dev, req);
		ret = BLK_STS_OK; 
	done:
		blk_mq_end_request (req, ret);
	printk(KERN_ALERT"%s() over.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	return ret;
}



/*
 * The direct make request version.
 */
static blk_qc_t sbull_make_request(struct request_queue *q, struct bio *bio)
{
	printk(KERN_ALERT"%s() begin.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	struct sbull_dev *dev = bio->bi_disk->private_data;
	int status;

	status = sbull_xfer_bio(dev, bio);
	bio->bi_status = status;
	bio_endio(bio);
	printk(KERN_ALERT"%s() over.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	return BLK_QC_T_NONE;
}

//请求处理模块（多队列模式）
static  blk_status_t sbull_mq_request(struct blk_mq_hw_ctx *hctx, const struct blk_mq_queue_data *bd)
{
	printk(KERN_ALERT"%s() begin.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	struct request *req = bd->rq;
	int sectors_xferred;//就算统计了已经转发的扇区数目好像也没什么用
	struct sbull_dev *dev = req->q->queuedata;
	blk_status_t  ret;//u32
	//void函数
	printk(KERN_NOTICE"the req is %p", req);
	blk_mq_start_request (req);

	if (blk_rq_is_passthrough(req)) {
		printk (KERN_NOTICE "Skip non-fs request\n");
		ret = BLK_STS_IOERR; //-EIO;
		goto done;
	}
	//请求转发 子模块
	sectors_xferred = sbull_xfer_request(dev, req);
	ret = BLK_STS_OK; //宏定义0
	//不管goto执不执行，都会执行到这里
	//把req从队列移除
	//如果不移除，那么就会不断执行第一条请求，死循环，ctrl c也没用
	done:
		blk_mq_end_request (req, ret);
	printk(KERN_ALERT"%s() over.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	return ret;
}
/*
 * Open and close.
 */

static int sbull_open(struct block_device *bdev, fmode_t mode)
{
	printk(KERN_ALERT"%s() begin.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	struct sbull_dev *dev = bdev->bd_disk->private_data;
	//移除定时器
	del_timer_sync(&dev->timer);
	spin_lock(&dev->lock);
	//如果没有用户，那么检查介质是否已改变
	if (! dev->users) 
	{
		//观察内核源码check-disk-change可能会调用revalidate函数
		//就是我们的sbull-revalidate
		check_disk_change(bdev);
	}
	dev->users++;
	spin_unlock(&dev->lock);
	printk(KERN_ALERT"%s() over.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	return 0;
}

static void sbull_release(struct gendisk *disk, fmode_t mode)
{
	printk(KERN_ALERT"%s() begin.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	//同样的，gendisk里面的private-data是void*
	struct sbull_dev *dev = disk->private_data;

	spin_lock(&dev->lock);
	printk(KERN_ALERT"%s() spin_lock locked!",__func__);
	dev->users--;
	//如果没有用户了，那么启动介质移除定时器！
	if (dev->users == 0) {
		//Linux内核使用全局变量jiffies记录系统自从启动以来的滴答数。
		//从现在的jiffies再推迟INVALIDATE_DELAY
		dev->timer.expires = jiffies + INVALIDATE_DELAY;
		//激活定时器
		add_timer(&dev->timer);
	}
	spin_unlock(&dev->lock);
	printk(KERN_ALERT"%s() spin_lock unlocked!",__func__);
	printk(KERN_ALERT"%s() over.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
}

/*
 * Look for a (simulated) media change.
 */
int sbull_media_changed(struct gendisk *gd)
{
	printk(KERN_ALERT"%s() begin.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	struct sbull_dev *dev = gd->private_data;
	
	printk(KERN_ALERT"%s() over.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	return dev->media_change;
}

/*
 * Revalidate.  WE DO NOT TAKE THE LOCK HERE, for fear of deadlocking
 * with open.  That needs to be reevaluated.
 */
int sbull_revalidate(struct gendisk *gd)
{
	printk(KERN_ALERT"%s() begin.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	struct sbull_dev *dev = gd->private_data;
	//如果设备介质改变
	//清空设备内存空间，模拟插入一张空白的磁盘。	
	if (dev->media_change) {
		dev->media_change = 0;
		memset (dev->data, 0, dev->size);
	}
	printk(KERN_ALERT"%s() over.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	return 0;
}

/*
 * The "invalidate" function runs out of the device timer; it sets
 * a flag to simulate the removal of the media.
 */
void sbull_invalidate(struct timer_list * ldev)
{
	printk(KERN_ALERT"%s() begin.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
        struct sbull_dev *dev = from_timer(dev, ldev, timer);
	spin_lock(&dev->lock);
	//如果又有用户来了或者data不空
	//那么就不media change
	if (dev->users || !dev->data) 
		printk (KERN_WARNING "sbull: timer sanity check failed\n");
	else
		dev->media_change = 1;
	spin_unlock(&dev->lock);
	printk(KERN_ALERT"%s() over.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
}

/*
 * The ioctl() implementation
 * 块层会先截取命令，识别不了的才会交给sbull的。
 * 不管用户是用整数arg还是指针arg，这里的arg都是unsigned long
 *
 * 如果在用户空间调用ioctl，可能会调用这里的sbull ioctl，实测可以
 * 传入cmd是666666，块设备层应该识别不了，就传到这儿了。
 */
int sbull_ioctl (struct block_device *bdev, fmode_t mode,
                 unsigned int cmd, unsigned long arg)
{
	printk(KERN_ALERT"%s() begin.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	long size;
	//内核定义的一个结构体，只有4个变量
	struct hd_geometry geo;
	//block_device is a kernel struct ,bd_disk is a member ,it is gendisk*
	//gendisk has a member ,a void *,it is private_data
	//so that is block_device* -- gendisk* -- void*
	struct sbull_dev *dev = bdev->bd_disk->private_data;

	switch(cmd) {
	    case HDIO_GETGEO:
		//几何结构
        	/*
		 * Get geometry: since we are a virtual device, we have to make
		 * up something plausible.  So we claim 16 sectors, four heads,
		 * and calculate the corresponding number of cylinders.  We set the
		 * start of data at sector four.
		 */
		size = dev->size*(hardsect_size/KERNEL_SECTOR_SIZE);
		geo.cylinders = (size & ~0x3f) >> 6;//相当于一个煮面的大小是2的6次方
		geo.heads = 4;//4个磁头
		geo.sectors = 16;
		geo.start = 4;
		//本质上也就是个memmove函数
		if (copy_to_user((void __user *) arg, &geo, sizeof(geo)))
			return -EFAULT;
		return 0;

	    case SAYHELLO_CYFCMD:
		printk(KERN_NOTICE"get cyf cmd sayhello, hello!!");
		return 0;
	}

	printk(KERN_ALERT"%s() over.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	return -ENOTTY; /* unknown command */
}


//linux 5.4 里面定义的是接收blk_mq_tag_set指针作为参数，然后返回整数
//这个返回值应该是硬件队列的下标
//这里直接返回0，反正只有一个硬队列
//内核源码 ：typedef int (map_queues_fn)(struct blk_mq_tag_set *set);
static int sbull_map_queues(struct blk_mq_tag_set *set)
{
	printk(KERN_ALERT"%s() begin.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	return 0;	
}

/*
 * The device operations structure.
 */
static struct block_device_operations sbull_ops = {
	.owner           = THIS_MODULE,
	.open 	         = sbull_open,
	.release 	 = sbull_release,
	.media_changed   = sbull_media_changed,  // DEPRECATED in v5.9
	.revalidate_disk = sbull_revalidate,
	.ioctl	         = sbull_ioctl
};

static struct blk_mq_ops mq_ops_simple = {
    .queue_rq = sbull_simple_request,
};

static struct blk_mq_ops mq_ops_full = {
    .queue_rq = sbull_full_request,
};

static struct blk_mq_ops sbull_mq_ops = {
	.queue_rq = sbull_mq_request,//不同模式的请求队列处理函数参数都是一样的
	.map_queues = sbull_map_queues,
};


/*
 * Set up our internal device.
 */
static void setup_device(struct sbull_dev *dev, int which)
{
	printk(KERN_ALERT"%s() begin.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	/*
	 * Get some memory.
	 */
	
	memset (dev, 0, sizeof (struct sbull_dev));
	//设备大小就是扇区数量乘扇区大小
	//扇区数量和扇区大小都是全局变量
	dev->size = nsectors*hardsect_size;
	dev->data = vmalloc(dev->size);
	if (dev->data == NULL) {
		printk (KERN_NOTICE "vmalloc failure.\n");
		return;
	}
	spin_lock_init(&dev->lock);
	
	/*
	 * The timer which "invalidates" the device.
	 */
	//初始化timer，注册回调函数
	//flag是0 flag随便穿一个数就行
        timer_setup(&dev->timer, sbull_invalidate, 0);

	/*
	 * The I/O queue, depending on whether we are using our own
	 * make_request function or not.
	 */
	switch (request_mode) {
	    case RM_NOQUEUE:
		//并没有真的建立一个保存请求的队列
		    dev->queue = blk_alloc_queue(GFP_KERNEL);
		if (dev->queue != NULL)
			//requset_queue	内部有个指向制造请求函数的指针
			//这里其实就是注册回调函数
			blk_queue_make_request(dev->queue, sbull_make_request);
		if (dev->queue == NULL)
			goto out_vfree;
		break;

	    case RM_FULL:
		dev->queue = blk_mq_init_sq_queue(&dev->tag_set, &mq_ops_full, 128, BLK_MQ_F_SHOULD_MERGE);
		printk(KERN_NOTICE"the nr_maps is %d", dev->tag_set.nr_maps);
		if (dev->queue == NULL)
			goto out_vfree;
		break;

	    default:
		printk(KERN_NOTICE "Bad request mode %d, using simple\n", request_mode);
        	/* fall into.. */
	
	    case RM_SIMPLE:
		dev->queue = blk_mq_init_sq_queue(&dev->tag_set, &mq_ops_simple, 128, BLK_MQ_F_SHOULD_MERGE);
		if (dev->queue == NULL)
			goto out_vfree;

	    case RM_MQ:
		printk(KERN_ALERT"cyf two-level multi-queue mode has been choosed...");
	
		dev->queue_depth = hw_queue_depth;
		//用于描述与存储器件相关的tag集合，抽象了存储器件的IO特征	
		dev->tag_set.ops = &sbull_mq_ops;//一些函数指针
		dev->tag_set.nr_hw_queues = nr_hw_queues;//硬队列数量
		dev->tag_set.queue_depth = hw_queue_depth;//硬队列深度
		dev->tag_set.numa_node = NUMA_NO_NODE;//宏定义-1
		dev->tag_set.cmd_size = sizeof(struct sbull_dev);//每个请求的额外数据
		dev->tag_set.flags = BLK_MQ_F_SHOULD_MERGE;
		dev->tag_set.driver_data = dev;//这是一个void类型的指针，每个请求的额外数据应该就是这个
		
		//blk-mq-alloc-tag-set会默认设置nr-maps为1
		if(blk_mq_alloc_tag_set(&dev->tag_set)){
			printk(KERN_ALERT"amazing, tag_set set failure!");
			goto out_vfree;
		}
		printk(KERN_NOTICE"the nr_maps is %d", dev->tag_set.nr_maps);
		//tag set可以是多个队列共享的
		dev->queue = blk_mq_init_queue(&dev->tag_set);
		if(dev->queue == NULL)
			goto out_vfree;

		break;

	    case RM_STACKBD:
		printk(KERN_NOTICE"come into cyf stackbd mode!");
		break;
	}
	//硬件扇区大小作为第一个参数放入请求队列
	//分配队列后立刻设置扇区大小	
	dev->queue->queuedata = dev;//queuedata 是void类型指针
	blk_queue_logical_block_size(dev->queue, hardsect_size);
	/*
	 * And the gendisk structure.
	 */
	//驱动程序不能自己初始化gendisk，必须调用alloc_disk
	dev->gd = alloc_disk(SBULL_MINORS);
	if (! dev->gd) {
		printk (KERN_NOTICE "alloc_disk failure\n");
		goto out_vfree;
	}
	dev->gd->major = sbull_major;
	dev->gd->first_minor = which*SBULL_MINORS;
	dev->gd->fops = &sbull_ops;//文件操作集合
	dev->gd->queue = dev->queue;
	dev->gd->private_data = dev;
	snprintf (dev->gd->disk_name, 32, "sbull%c", which + 'a');//名字
	set_capacity(dev->gd, nsectors*(hardsect_size/KERNEL_SECTOR_SIZE));
	add_disk(dev->gd);
	printk(KERN_ALERT"%s() over.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	return;
	
	//dev->data是用vmalloc分配的
  out_vfree:
	if (dev->data)
		vfree(dev->data);
}

static int __init sbull_init(void)
{
	printk(KERN_ALERT"%s() begin.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	int i;
	//向内核注册自己
	//如果传入的参数是0，内核将返还新的主设备号
	//主设备号对应驱动程序，而次设备号对应这个驱动下的某个具体设备
	sbull_major = register_blkdev(sbull_major, "sbull");
	printk(KERN_NOTICE"get new major number:%d", sbull_major);	
	//如果返回负值，则表示出现了错误
	//书上写的负值，但是这里却包括了0
	if (sbull_major <= 0) {
		printk(KERN_WARNING "sbull: unable to get major number\n");
		return -EBUSY;//device or resource busy
		//EBUSY是一个宏定义，是16
		//这里返回的也就是-16
	}
	/*
	 * Allocate the device array, and initialize each one.
	 */
	//kmalloc is the normal method of allocating memory for objects smaller than page size in the kernel.
	//GFP_KERNEL - Allocate normal kernel ram. May sleep.
	Devices = kmalloc(ndevices*sizeof (struct sbull_dev), GFP_KERNEL);
	if (Devices == NULL)
		goto out_unregister;
	
	//循环安装块设备
	for (i = 0; i < ndevices; i++) 
		setup_device(Devices + i, i);
    
	printk(KERN_ALERT"%s() over.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	return 0;

  out_unregister:
	printk(KERN_ALERT"%s() out register! The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	//注销块设备
	//第二个参数是干啥的？
	unregister_blkdev(sbull_major, "sbd");
	return -ENOMEM;//out of memory 宏定义12
}

//模块退出时必须把初始化函数里面分配的内存全部释放！
//否则，linux重新启动后会发现之前的某些东西残留在系统里面
static void sbull_exit(void)
{
	printk(KERN_ALERT"%s() begin.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
	int i;
	for (i = 0; i < ndevices; i++) {
		struct sbull_dev *dev = Devices + i;

		del_timer_sync(&dev->timer);
		if (dev->gd) {
			//卸载磁盘函数
			//gendisk是一个引用计数结构，del_gendisk删除gendisk中的最终计数
			del_gendisk(dev->gd);
			//负责处理引用计数
			//put_disk内部仍然是调用kobject_put(),这里不再赘述
			put_disk(dev->gd);
		}
		if (dev->queue) {
			if (request_mode == RM_NOQUEUE)
				//为什么要处理引用计数呢？
				//blk_put_queue的定义很简单，就是调用kobject_put去处理queue里面的kobject对象
				//kobject_put()
				//Decrement refcount for object.
				//Decrement the refcount, and if 0, call kobject_cleanup().
				blk_put_queue(dev->queue);
			else
				//删除块设备请求队列
				//把请求队列返还给系统
				//调用这个函数后，驱动程序将不再得到这个队列的请求
				blk_cleanup_queue(dev->queue);
		}
		//设备实际保存数据的数组是用vmalloc分配的
		//所以用vfree
		if (dev->data)
			vfree(dev->data);
	}
	//注销设备
	//这里的第二个参数怎么和init函数里的不一样？
	unregister_blkdev(sbull_major, "sbull");
	//Devices使用kmalloc分配的，所以使用kfree释放
	kfree(Devices);
	printk(KERN_ALERT"%s() over.The porcess is \"%s\" (pid %i)",__func__, current->comm, current->pid);
}
	
module_init(sbull_init);
module_exit(sbull_exit);
