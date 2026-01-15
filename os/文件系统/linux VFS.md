linux 虚拟文件系统 (VFS, virtual file system) 是接口层, 将 [linux 文件系统](linux%20文件系统.md)具体实现与操作系统服务剥离开. VFS 表示为统一的**树形**目录结构, 新文件系统通过**挂载 (mount)** 的方式装载到 VFS 的 `/mnt` 目录中. 

VFS 将不同 I/O 对象的接口统一, 都通过 `struct file` 与其内部的 `struct file_operations` 虚函数表表达. VFS 负责将路径解析为对应的内部对象.
VFS 支持的文件类型包括:
- file (f), directory (d), symlink (l) --> fs 
- executable (x)
- empty (e)
- socket (s)
- pipe (p) 
- char-dev (c): 字符设备, 一次性读取
- block-dev (b): 块设备, 支持随机存取


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
	...
	__poll_t (*poll) (struct file*, ...);
	long (*unlocked_ioctl) (struct file*, ...);
	int (*mmap) (struct file*, struct vm_area_struct*);
	int (*open) (struct inode *, struct file *);
	int (*flush) (struct file*, ...);
	...
	int (*lock) (struct file*, int, struct file_lock *);
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

| flags       | 含义             |
| ----------- | ---------------- |
| `O_RDONLY`  | 只读             |
| `O_WRONLY`  | 只写             |
| `O_RDWR`    | 读写. 注意 `O_RDONLY, O_WRONLY, O_RDWR` 只能使用一个             |
| `O_APPEND`  | 追加             |
| `O_CREAT`   | 创建一个文件. 如果文件已存在, 则会发生错误.     |
| `O_EXEC`    |                  |
| `O_NOBLOCK` | 以非阻塞方式打开 |
| `O_TRUNC`   | 如果文件已存在, 则删除文件的内容                 |

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
open("test", O_CREAT, 10 705);

open("test", O_CREAT, S_IRWXU | S_IROTH | S_IXOTH | S_ISUID);
```

#### 文件描述符

文件描述符数量有上限 `RLIMIT_NOFILE` 限制. 系统能分配的最大文件描述符数由  `sizeof(int)` 限制, 记录在 `/pro/sys/fs/file-max`; 单个进程的最大文件描述符数记录在 `/proc/sys/fs/nr_open`.

`Too may open files` 错误有两种:
- `EMFILE` per-process limit : 可能是超上限, 也可能是资源泄漏 (没有关闭)
- `ENFILE` system-wide limit 

使用 `lsof -p <pid>` 查看进程的文件描述符, 用 `ulimit` 提升进程 FD 限制. 当然, 更推荐的方法, 是使用 IO 复用.

```cpp
class FD {
	int fd_;

public:
	explicit FD(int fd = -1) noexcept : fd_(fd) {}
	
	// 关闭拷贝语义
	// 允许移动语义
	
	~FD() { reset(); }
	void reset(int newfd = -1) noexcept {
		if (fd_ != -1)
			::close(fd_);
		fd_ = newfd;
	}
	int fd() const noexcept { return fd_; }
	explicit operator bool() const noexcept { return fd_ != -1; }
}
```

### read/write 

```c
// 从 fd 读取 length 个字节到 buf 缓冲区, 返回实际读取的字节数.
int read(int fd, const void *buf, size_t length);
int write(int fd, const void *buf, size_t length);
```

## lseek 

对于随机文件, 可以在随机指定位置读写. 

```c
// 将文件读写指针相对 whence 移动 offset 个字节. 
// 成功时, 返回文件指针相对于文件头的位置.
int lssek(int fd, offset_t offset, int whence);
```

`whence` 可以是如下值:
- `SEEK_SET`: 相对文件开头
- `SEEK_CUR`: 相对文件读写指针的当前位置
- `SEEK_END`: 相对文件末尾

返回文件长度:
```
file_len = lseek(fd, 0, SEEK_END);
```

### gendisk

```c
struct hd_struct {
    long start_sect;
    long nr_sects;
};

struct gendisk {
    int major;               /* major number of driver */
    const char *major_name;  /* name of major driver */
    int minor_shift;         /* number of times minor is shifted to
                                get real minor */
    int max_p;               /* maximum partitions per device */
    int max_nr;              /* maximum number of real devices */

    void (*init)(struct gendisk *); 
                             /* Initialization called before we 
                                do our thing */
    struct hd_struct *part;  /* partition table */
    int *sizes;              /* device size in blocks, copied to 
                                blk_size[] */
    int nr_real;             /* number of real devices */

    void *real_devices;      /* internal use */
    struct gendisk *next;
};
```

## STDC 文件系统调用

标准库的优点就是**跨平台一致性**, 语义在不同平台 (Linux, Windows, VxWorks) 之间保持一致.

```c
FILE *fopen(const char *path, const char *mode);
```

详见 [stdio.h](../../langs/c/stdio.md).