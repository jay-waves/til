linux 虚拟文件系统 (VFS, virtual file system) 是接口层, 将 [linux 文件系统](linux%20文件系统.md)具体实现与操作系统服务剥离开. VFS 表示为统一的**树形**目录结构, 新文件系统通过**挂载 (mount)** 的方式装载到 VFS 的 `/mnt` 目录中. 

VFS 将不同文件系统实现的接口统一, 使 linux 具有支持不同环境的能力, 透明地实现内核功能. 定义和容纳具体文件系统的设备称为**块设备 (block device).**

VFS 在两个主要目标上做 trade-off:
1. 高效访问文件.
2. 确保文件数据正确.

## Linux 文件系统调用


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

标准库的优点就是**跨平台一致性**, 不会在不同平台 (Linux, Windows, VxWorks) 之间有所区别.

```c
FILE *fopen(const char *path, const char *mode);
```

详见 [stdio.h](../../CodeGlyph/C/标准库/stdio.md).