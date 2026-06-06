
## Linux 文件系统调用

```c
/*
	include/linux/fs.h
*/
struct file {
	...
	struct file_operations *f_op; 
	...
};

struct file_operations {
	...
	loff_t (*llseek) (struct file *, loff_t, int);
	ssize_t (*read) (struct file*, char __user *, size_t, loff_t *);
	ssize_t (*write) (struct file*, ...);
	ssize_t (*read_iter) (struct kiocb*, struct iov_iter*); // preferred 
	ssize_t (*write_iter) (struct kiocb*, struct iov_iter*); // perferred 
	...
	__poll_t (*poll) (struct file*, ...);
	int (*ioctl) (struct inode*, struct file*, unsigned int, unsigned long); 
	// unlocked_ioctl
	// compat_ioctl h
	long (*unlocked_ioctl) (struct file*, ...);
	int (*mmap) (struct file*, struct vm_area_struct*);
	int (*open) (struct inode *, struct file *);
	int (*flush) (struct file*, fl_owner_t);
	int (*release) (struct file*, struct file*);
	int (*fsync) (struct file *, struct dentry*, int datasync);
	int (*fasync) (struct file*, struct file_lock*);
	int (*lock) (struct file*, int, struct file_lock*);
	
	unsigned long (*get_unmapped_area) (struct file*, ...);
	.
	int (*flock) (struct file*, int, struct file_lock*);
	ssize_t (*splice_write) (struct pipe_inode_info*, struct file*, loff_t *, size_t, unsigned int);
	ssize_t (*splice_read) (struct file*, loff_t *, struct pipe_inode_info*, size_t, unsigned int);
	...
};
```

### create

```c
// mode & umask 共同决定文件的最终权限
int create(const char *filename, mode_t mode); 
// 等价于 create:
int open(pathname, O_CREAT | O_WRONLY | O_TRUNC, mode);

int umask(int newmask); // 用于去掉一些文件的存取 (rwx) 权限. 返回旧的 umask
```

### open / close 

使用系统调用 `open()` 打开文件, 返回文件描述符. 内核为每个进程建立了一个*打开文件表*, 打开文件后, 该文件的描述符被存储于打开文件表中. 

```c
int open(const char *pathname, int flags);
int open(const char *pathname, int flags, mode_t mode);

int close(int fd);
```

| flags       | 含义                                                 |
| ----------- | ---------------------------------------------------- |
| `O_RDONLY`  | 只读                                                 |
| `O_WRONLY`  | 只写                                                 |
| `O_RDWR`    | 读写. 注意 `O_RDONLY, O_WRONLY, O_RDWR` 只能使用一个 |
| `O_APPEND`  | 追加                                                 |
| `O_CREAT`   | 创建一个文件. 如果文件已存在, 则会发生错误·.         | 
| `O_EXEC`    |                                                      |
| `O_NOBLOCK` | 以非阻塞方式打开                                     |
| `O_TRUNC`   | 如果文件已存在, 则删除文件的内容                     |

使用 `O_CREAT` 时, 必须:

| mode      | 含义                     |
| --------- | ------------------------ |
| `S_IRUSR` | 用户可读                 |
| `S_IWUSR` | 用户可写                 |
| `S_IXUSR` | 用户可执行               |
| `S_IRWXU` | 用户可读, 可写, 可执行   |
| `S_IRGRP` | 用户组可读               |
| `S_IWGRP` | 用户组可写               |
| `S_IXGRP` | 用户组可执行             |
| `S_IRWXG` | 用户组可读, 可写, 可执行 |
| `S_RIOTH` | 其他人可读 (其他略)      |
| `S_ISUID` | 设置用户的执行 ID          |
| `S_ISGID` | 设置用户组的执行 ID                         |

Linux 用五个数来表示文件权限: 比如 `10705`
- 第一个数字: 是否设置用户 ID. `0, 1`
- 第二个: 是否设置用户组 ID 
- 第三个: 用户权限位. `0, 1, 2, 4, 5, 7`
- 第四个: 用户组权限位.
- 第五个: 其他人权限位.

以下两个调用等价:

```c
open("path/to/test", O_CREAT, 10 705);

open("path/to/test", O_CREAT, S_IRWXU | S_IROTH | S_IXOTH | S_ISUID);
```

### read/write 

```c
// 从 fd 读取 length 个字节到 buf 缓冲区, 返回实际读取的字节数.
int read(int fd, const void *buf, size_t length);
int write(int fd, const void *buf, size_t length);
```

从 Linux3.9+ 开始，读写路径逐渐切换到 `struct iov_iter` 上，即，使用以下接口以及结构体 [`iov_iter`](../io/linux-iov_iter.md)

```c
void read_iter(struct kiocb *, struct iov_iter*);
void write_iter(struct kiocb *, struct iov_iter*);
```

原本的 `aio_read/aio_write/readv/writev` 接口都在新版本被删除了，事实上 `write/read`也在废除边缘。

文件夹迭代器也更新为：

```c
iterate_shared(struct file*, struct dir_context*);
```

### lseek 

对于随机文件, 可以在随机指定位置读写. 

```c
// 将文件读写指针相对 whence 移动 offset 个字节. 
// 成功时, 返回文件指针相对于文件头的位置.
int llseek(int fd, offset_t offset, int whence);
```

`whence` 可以是如下值:
- `SEEK_SET`: 相对文件开头
- `SEEK_CUR`: 相对文件读写指针的当前位置
- `SEEK_END`: 相对文件末尾

返回文件长度:
```
file_len = lseek(fd, 0, SEEK_END);
```

### mmap 

```c
mmap(struct file*, struct vm_area_struct*);
mmap_prepare(struct vm_are_struct*);
```

用户空间直接映射 PageCache，不再需要从内核内存拷贝到用户内存。

详见 [os/libc/mem-alloc](../libc/mem-alloc.md)

### poll 

```c
poll(struct file*, struct poll_table_struct*);
```

详见 [os/io/poll](../io/poll.md), 用于 `poll` 和 `select` 系统调用。

### ioctl 

详见 [os/io/drivers/ioctl](../io/drivers/ioctl.md)

### splice

用于从管道向文件传输数据。

```c
splice_read();
splice_write();

copy_file_range();
remap_file_range();
```

内核零拷贝接口，详见 [os/io/splice](../io/splice.md)

### release & flush 

当 `struct file` 对象的引用数为零时，调用 `release` 

### fsync

`fsync` 用于将内存中文件数据与存储介质同步。`fasync` 用于触发由信号控制的输入输出。

```c
fsync(struct file*, loff_t, loff_t, int datasync);
```

## STDC 文件系统调用

标准库的优点就是**跨平台一致性**, 语义在不同平台 (Linux, Windows, VxWorks) 之间保持一致.

```c
FILE *fopen(const char *path, const char *mode);
```

详见 [stdio.h](../../langs/c/stdio.md).

## 参考

[Overview of the Linux Veritual File System](https://docs.kernel.org/filesystems/vfs.html)

[the file_operations structure gets smaller, lwn.net](https://lwn.net/Articles/972081/#Comments)