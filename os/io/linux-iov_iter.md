`iov_iter` 描述本次请求要操作的内存缓冲区域；`kiocb` 表示单次 I/O 请求，包括文件和偏移上下文。

```c

enum iov_iter_type {
		ITER_IOVEC, // user space 
		ITER_KVEC, // kernel space 
		ITER_BVEC, // block 
};

struct iov_iter {
	int  type; 
	sizej_t iov_offset;
	size_t count;
	union {
		const struct iovec *iov;
		const struct bio_vec *bvec;
	};
	unsigned long nr_segs;
};

enum ki_flag {
	IOCB_DIRECT, // 绕过页缓存
	IOCB_NOWAIT, // 必须阻塞时，直接返回 -EAGAIN
	IOCB_DSYNC,  
	IOCB_SYNC,  
	IOCB_APPEND,
	IOCB_HIPRI,
	IOCB_NOIO,
	IOCB_ALLOC_CACHE,
	IOCB_WAITQ
};

struct kiocb {
	struct file *ki_filp; 
	loff_t ki_ops; 
	
	int ki_flags;
	u16 ki_ioprio; // IO 优先级

	struct aio_context *ki_ctx; 
	void (*ki_complete)(struct kiocb *iocb, long ret); // user-defined callback 
	void *private; // user defined data 
	unsigned flags;
};
```

## 场景

### 同步读写

user space: 

```c
read(fd, buf, 4096);
```

kernel space:

```c
init_sync_kiocb(&kiocb, file);
kiocb.ki_pos = file->f_pos;

// 将用户缓冲区封装为 iov_iter，内核不能直接访问用户态内存
iov_iter_ubuf(&iter, ITER_DEST, buf, 4096); 

ret = file->f_op->read_iter(&kiocb, &iter);

if (ret > 0)
	file->f_pos = kiocb.ki_pos;
```

### 批量读写 

用户态

```c
struct iovec iov[3] = {
	{ .iov_base = buf1, .iov_len = 1024 },
	{ .iov_base = buf2, .iov_len = 2048 },
	{ .iov_base = buf3, .iov_len = 4096 },
};

iter.type = ITER_IOVEC;
iter.iov = iov;
iter.count = 1024 + 2048 + 4096;

while (iov_iter_count(iter)) {
	n = copy_some_data(iter); // ...
	iov_iter_advance(iter, n);
	iocb->ki_pos += n;
}
```

