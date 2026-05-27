
以一些常见设备为例：

| 主设备号 | 从设备号 | 文件         | 含义             |
| -------- | -------- | ------------ | ---------------- |
| 1        | 1        | /dev/mem     | 物理内存         |
| 1        | 2        | /dev/kmem    | 内核虚拟地址空间 |
| 1        | 3        | /dev/null    |                  |
| 1        | 4        | /dev/port    | I/O 端口         |
| 1        | 5        | /dev/zero    |                  |
| 1        | 8        | /dev/random  |                  |
| 5        | 0        | /dev/tty     |                  |
| 5        | 1        | /dev/console |                  |

```c
#include <cdev.h>
struct cdev {
        struct kobject kobj;
        struct module *owner;              // 提供该驱动的 KModule
        const struct file_operations *ops; // 方法
        struct list_head list;
        dev_t dev;                         // 设备号
        unsigned int count;
}; 
```

假设用户态通过 VFS 打开该设备 `open("/dev/xxx")` ，VFS 通过 inode 的文件类型重定向到 `.open = chrdev_open()`，调用后获取 `cdev` 对象，并用 `cdev->ops` 替换 VFS 中的 `file->f_op` ，最终，调用 `file->f_op->open()` 完成操作。

以 `/dev/null` 为例，它注册的 fops 操作是：

```c
static struct file_options null_fops = {
		.llseek = null_lseek,
		.read   = read_null, 
		.write  = write_null, 
		.splice_write = splice_write_null,
};
```
