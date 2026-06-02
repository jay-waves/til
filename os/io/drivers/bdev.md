基于 Linux6.14

硬件设备的*扇区（sector）*大小一般是 512B，而软件*块（block）*大小则一般是 4KB，因为 4KB 正好是内存页的大小。

块设备的内部接口比较复杂：
* 需要支持随机寻址，并且处理块区的对齐问题
* 按块读取底层数据，需要解决缓存、容错和并发一致性问题。


用户态通过 VFS 打开块设备 `open("/dev/xxx")` ，VFS 通过 inode 的文件类型重定向到 `.open = blkdev_open()`，调用后获取 `gendisk`，
* 是主设备，调用 `gendisk->fops->open()`
* 是分区，插入一个 `block_device` 

```c
// 表示一整块硬盘设备，如 sda, nvme0n1
struct gendisk 
{
	int major; 
	int first_minor;
	int minors; 
	
	struct block_device * part0;  // whole-disk bd
	struct xarray* part_tbl;      // partition bd (index i --> struct block_device)
	struct block_device_operations * fops;
	
	struct request_queue *queue;
	
	...
};

// 表示一个可用硬盘设备（通常指一个硬盘的某个分区，如 sda1）
struct block_device
{
		bd_start_sect; 
		bd_nr_sectors; 
		struct gendisk* bd_disk;
		struct request_queue* bd_queue;
		
		bd_dev dev_t;
		bd_meta_info; 
};

/// 一个设备上的 IO 动作队列
struct request_queue 
{
		disk; // back pointer to gendisk 
		queuedata;
		mq_ops; 
		tag_set; 
};
```

```ascii
                 +------------------+
                 |   struct gendisk |
                 |   "sda"          |
                 +--------+---------+
                          |
          +---------------+----------------+
          |                                |
          v                                v
+---------------------+          +----------------------+
| part0               |          | part_tbl             |
| whole disk bdev     |          | partitions           |
| /dev/sda            |          | /dev/sda1 /dev/sda2  |
+----------+----------+          +----------+-----------+
           |                                |
           +---------------+----------------+
                           |
                           v
                +----------------------+
                | struct block_device  |
                +----------+-----------+
                           |
                           v
                +----------------------+
                | struct request_queue |
                +----------------------+
```


### BIO 

BIO 描述了l上层对 Block 设备的一次 IO 请求。为了支持并发，多个 BIO 可能被合并为一个 `struct request`，因此 `struct request` 才是被调度的 IO 动作。

```c

// 一个上层请求的 IO 动作
struct bio 
{
		struct bio  *bi_next;
		struct block_device *bi_bdev;  // e.g. /dev/sda1
		blk_opf_t bi_opf;  // operation: read, write, flush...
		unsigned short  bi_flags;
		blk_status_t  bi_status;
		
		
		struct bio_vec {
			struct page *bv_page;
			unsigned int bv_offset;
			unsigned int bv_len;
		} bi_io_vec[]; // memory segment array 
		unsigned short bi_vcnt; // number of elements in bio_vec[]
		
		struct bvec_iter {
			bi_sector;
			bi_size; 
		} bi_iter;
		
		bio_end_io_t; 
};

struct request 
{
		struct request_queue *q; // belonger 
		struct blk_mq_ctx    *mq_ctx; // software queue conctex[]
		struct blk_mq_hw_ctx *mq_hctx; // hardware queue contex[]
		
		blk_opf_t  cmd_flags; // flags + operations 
		int tag; // blk-mq hardware tag 
		
		unsigned int timeout; 
		
		struct bio *bio; // first bio in request 
		struct bio *biotail; // last bio in request 
		
		struct list_head queuelist; 
		
		struct block_device *part;  
		
		struct blk_mq_tags; // tag 是请求的实际编号
		struct blk_mq_ops; // driver callbacks 
		
		sector_t     __sector; 
		unsigned int __data_len;
		
		enum mq_rq_state state; // idle / in-flight / complete 
		atomic_t ref; // ref count 
		
};

struct blk_mq_ops.queue_rq() 
```

`mq_ctx[]` 对应 CPU 侧的并发队列，通常 N 个 CPU 对应 N 个请求队列，而 `mq_hctx[]` 对应设备侧的队列，blk-mq 负责两者间的 `request` 映射（派发）。

### IO Scheduler 

内核中将 IO 调度器统称为 *elevartor* ，不过具体的调度方式总在变化。IO 调度器负责请求队列的重排、合并和分发。

Linux 6.x 的 IO 调度器主要是 `blk-m` ，即 block layer multi-queue request。特点是高并发。它支持如下调度方式：
* none, 尽快分发（先来先服务），能合并请求，但无法重排
* mq-deadline 
* bfq 
* kyber 

```c

struct elevator_type 
{
		struct list_herad list; 
		struct elevator_ops ops;
		...
}

struct blk_mq_ops {
	void (*queue_rq)();
}
```

### 块设备操作

```c

struct block_device_operations {
		int (*open) (struct inode *, struct file*);
		int (*release) (struct inode *, struct file*);
		int (*ioctl) (struct inode *, struct file*, unsigned, unsigned long);
};

```
