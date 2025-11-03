
## Preparation

- turn off modversioning (`CONFIG_MODVERSIONS`)
- disable UEFI SecureBoot, to enable to boot with third-party software.

安装内核头文件:
```shell
# on Debian
sudo apt update
sudo apt-cache search linux-header-`uname -r`
sudo apt install kmod linux-headers-5.4.0-80-generic # just an example

# on Arch
sudo pacman -S linux-headers
```

头文件目录为 `/lib/modules/<kernel_version>/build/include`. 

对于 WSL2, 默认内核不允许手动加入新模块, 所以没有 `/lib/modules` 目录, 需要[自行编译和安装新内核](Container/wsl%20更新内核.md). [^1] 这样做的另一个好处是, 编译内核和内核模块使用的环境是一致的.

[^1]: 详见问题 [wsl2 doesnot have /lib/modules](https://unix.stackexchange.com/questions/594470/wsl-2-does-not-have-lib-modules#:~:text=For%20those%20that%20need%20to%20load%20modules%20on,5%20Restart%20WSL.%20Your%20module%20should%20be%20loaded.).


## Hello World

> "It is unclear what the outcomes are for those who deviate from this 
> tradition, but is seemes prudent to adhere to it."

```c
#include <linux/module.h> /* Needed by all modules */
#include <linux/printk.h> /* Needed for pr_info() */

int init_module(void)
{
    pr_info("Hello world 1.\n");

    /* A non 0 return means init_module failed; module can't be loaded. */
    return 0;
}

void cleanup_module(void)
{
    pr_info("Goodbye world 1.\n");
}

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("YJW");
MODULE_DESCRIPTION("a sample driver?");
```

调用 `insmod` 插入模块时, 需要 `init_module()`, 移除模块时则需要 `cleanup_module()` 调用. 

在后期内核版本, `init_module()` 和 `cleanup_module()` 可以使用 `module_init` 和 `module_exit` 宏来重命名. 但具体的定义要出现在调用这些宏之前.

```c
#include <linux/init.h>   /* Needed for the macros */
#include <linux/module.h> /* Needed by all modules */
#include <linux/printk.h> /* Needed for pr_info() */

static int __init hello_2_init(void)
{
    pr_info("Hello, world 2\n");
    return 0;
}

static void __exit hello_2_exit(void)
{
    pr_info("Goodbye, world 2\n");
}

module_init(hello_2_init);
module_exit(hello_2_exit);
```

### Makefile

for wsl2:

```makefile
obj-m := hello.o
KERNEL := /path/to/WSL2-Linux-Kernel
PWD := /path/to/your/modules

all:
    make -C $(KERNEL) M=$(PWD) modules

clean:
    make -C $(KERNEL)$ M=$(PWD)$ clean

test:
    # tell make to ignore error in case the module isn’t loaded.
    -sudo rmmod hello.ko 
    # Clear the kernel log without echo
    sudo dmesg -C
    # Insert the module
    sudo insmod hello.ko
    # Display the kernel log
    dmesg
unload:
    sudo rm /dev/lkm_example
    sudo rmmod lkm_example
```

编写好 `Makefile` 后, 使用 `make` 来完成编译. 当前文件夹会出现 `hello-1.ko`, 使用 `modinfo` 来查看该模块的信息. 

- 加载内核模块: `sudo insmod hello-1.ko`
- 列出已加载的模块: `sudo lsmod`. 实际是读取 `/proc/modules` 文件, 部分信息也在 `/sys/module` 目录下
- 接着, 删除模块: `sudo rmmod hello_1`
- 当前模块的依赖关系存放在: `/lib/modules/<kernel-version>/modules.dep`

> 关于内核详细机制, 查看:
> - Documentation/kbuild/modules.rst
> - Documentation/kbuild/makefiles.rst

## 模块参数

使用 `module_param(name, type, rw_permission)` 来为模块定义一个参数. 如果不传递参数, 会使用模块内置的缺省值. 如果是内置模块, 需要在 bootloader 的 bootargs 中传递参数. 

用 insmod/kmodprobe 插入模块时, 可以传递参数: `$ insmod mymodule.ko myvar=5`

```c
#include <linux/moduleparam.h>

int myint = 3;
module_param(myint, int, 0);
MODULE_PARM_DESC(myint, "An interger"); // description for this paramter

// array arguments
int myintarray[2];
// not interested in count
module_param_array(myintarray, int, NULL, 0); 

short myshortarray[4];
int count;
// put count int "count"
module_param_arrya(myshortarray, short, &count, 0); 

// module_param_string();
```

模块加载后, `/sys/module/` 目录下会出现一个相关目录.

## 模块加载和卸载

加载其他内核模块:

```c
request_module(module_name);
```

在内核中, 标识为 `__init` 的函数如果直接编译入内核, 成为内核镜像的一部分, 在链接时都会放在 `.init.text` 区段里. 同时, 所有函数的函数指针都保存在 `.initcall.init`, 初始化内核时会依次调用, 然后释放这些部分 (`.init.text, .initcall.init`) 的内存.

```c
#define __init __attribute__ ((__section__(".init.text")))
```

宏 `__initdata` 定义只有初始化时使用的数据, 初始化结束后即释放内存.

## 模块导出符号

`/proc/kallsysm` 文件中包含了*内核符号表*. 模块使用如下宏, 来导出符号到内核符号表中:

```c
EXPORT_SYMBOL(sym_name);
EXPORT_SYMBOL_GPL(sym_name); // GPL License
```

如果一个符号是用 `EXPORT_SYMBOL_GPL()` 导出, 就不可以被非 GPL 发布的模块使用. 注意, 有相当多内核模块符号都是用 GPL 导出的. Linux 不保证模块导出的符号的稳定性.

模块计数管理接口:
```c
int try_module_get(struct module *mod); // 
void module_put(struct module*);        // 增加模块的使用计数, 若返回 0 则表示调用失败.
```


