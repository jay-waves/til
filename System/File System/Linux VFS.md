linux 虚拟文件系统 (VFS, virtual file system) 是接口层, 将[linux 文件系统](linux%20文件系统.md)具体实现与操作系统服务剥离开. VFS 表示为统一的**树形**目录结构, 新文件系统通过**挂载 (mount)** 的方式装载到 VFS 的 `/mnt` 目录中. 

VFS 将不同文件系统实现的接口统一, 使 linux 具有支持不同环境的能力, 透明地实现内核功能. 定义和容纳具体文件系统的设备称为**块设备 (block device).**

VFS 在两个主要目标上做 trade-off:
1. 高效访问文件.
2. 确保文件数据正确.

```c
open()
write()
read()
close()
```

## open

使用系统调用 `open()` 打开文件, 返回文件描述符. 内核为每个进程建立了一个*打开文件表*, 打开文件后, 该文件的描述符被存储于打开文件表中.

```c
int open()
```


## gendisk

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