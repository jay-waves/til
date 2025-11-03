## Linux

除系统调用外, 内核对外的接口还有:
- VFS: `/proc, /sys`
- eBPF

### x86

| eax | name    | C prototype                                              | description  | args description                                                              |
| --- | ------- | -------------------------------------------------------- | ------------ | ----------------------------------------------------------------------------- |
| 1   | exit    | `void _exit(int status)`                                 | process exit | ebx: exit code                                                                |
| 2   | fork    | `pid_t fork(void)`                                       | process copy |                                                                               |
| 3   | read    | `ssize_t read(int fd, void *buf, size_t count)`          |              | ebx: file description </br> ecx: read buffer </br> edx: read size             |
| 4   | write   | `ssize_t write(int fd, const void *buf, size_t count)`   |              |                                                                               |
| 5   | open    | `int open(const char *pathname, int flags, mode_t mode)` |              | ebx: file path </br> ecx: permission (rwxp) </br> edx: if not existed, create |
| 6   | close   | `int close(int fd)`                                      |              | ebx: file description                                                         |
| 7   | waitpid | `pid_t waitpid( pid_t pid, int *status, int options)`    |              | ebx: pid </br> ecx: ptr to exit code </br> edx: mode                          |
| 8   | create  | `int create(const char *pathname, mode_t mode)`          | create file  | ebx: file path </br> ecx: mode                                                |

### amd64

`x86_64` 中, Linux 的系统调用号被重新分配.

### ioctl 

### netlink 

### ptrace 

### mmap

## POSIX

`/usr/include/unistd.h` 包含 Unix 系统调用标准库, 扩展 POSIX API 定义的标准系统调用. 是 POSIX 标准库的一部分, 如 `unistd.h, fcntl.h, sys/types.h`.

Linux 内核的系统调用需要从用户态切换到用户态, 因此需要专用的系统调用指令: `int 0x80, sysenter, syscall`. 通过调用号来派发到具体内核函数. POSIX 系统调用实际由 libc 实现并提供, libc 对内联汇编进行封装.


| 分类       | 函数                                         | 作用                           |
| ---------- | -------------------------------------------- | ------------------------------ |
| Process    | `execl(), ...`                               | 执行文件                       |
|            | `fork()`                                     |                                |
|            | `setpgid(), getpgid(), setpgrp(), getpgrp()` | 进程组管理                     |
|            | `getpid(), getppid()`                        |                                |
|            | `getsid(), setsid`                           | session                        |
|            | `nice()`                                     | 获取进程优先级                 |
|            | `sleep()`                                    | 睡眠 (秒)                      |
|            | `exit()`                                     |                                |
| FileSystem | `chdir(), fchdir()`                          | 修改工作目录                   |
|            | `chown(), ...`                               | 修改属主                       |
|            | `close()`                                    | 释放文件描述符                 |
|            | `access()`                                   | 检查用户访问某文件的权限       |
|            | `dup()`                                      | 复制文件描述符                 |
|            | `fsync()`                                    | 同步某文件??                   |
|            | `getcwd()`                                   | 获取当前工作目录               |
|            | `isatty()`                                   | 检查一个文件描述符是否指向终端 |
|            | `link()`                                     | 重命名文件                     |
|            | `lockf()`                                    |                                |
|            | `pipe()`                                     | 创建管道                       |
|            | `pread(), pwrite()`                          | 根据给定偏移量读写文件         |
|            | `read()`                                     | 从文件描述符读文件             |
|            | `readlink()`                                 | 从符号链接读文件               |
|            | `rmdir()`                                    | 删除文件夹                     |
|            | `symlink()`                                  | 创建链接 (文件别名)            |
|            | `sync()`                                     | 将内存中文件系统缓存写入硬盘   |
|            | `ttyname()`                                  |                                |
|            | `write()`                                    | 向一个文件描述符写入           |
|            | `unlink`                                     |                                |
| User/Group | `getgid()`                                   |                                |
|            | `getuid()`                                   |                                |
|            | `getlogin()`                                 | 获取当前用户名                 |
|            | `setuid(), setgid()`                         |                                |


## Add User Syscalls

**操作系统实验ch06, 拦截并修改某一系统调用**,   
模块替代法在WSL2中不可用 (WSL共享部分Windows系统调用, Windows不允许替代系统调用),   
变通使用内核编译法直接添加一个系统调用.

### 内核编译法

源码使用的是`linux-5.15.110`, 自己从官网下.

#### 1 Add System Call:

修改`/linux-5.15.110/kernel/sys.c`, 添加:  

```c
SYSCALL_DEFINE0(helloyjw){
        printk("Hello YuJiaWei *__*");
        return 1;
}
#endif /* CONFIG_COMPACT */
```

#### 2 Add Function Declaration

修改`/linux-5.15.110/include/linux/syscalls.h`, 并在 `#ifndef CONFIG_ARCH_HAS_SYSCALL_WRAPPER`开头添加:  

```c
/* adding for OS lab */
asmlinkage int sys_helloyjw(void);
```


#### 3 Add System Call Number

修改 sys_call_table, 位于`/linux-5.15.110/arch/x86/entry/syscalls/syscall_64.tbl`, 添加一行:

```tbl
439  64  helloyjw  sys-helloyjw
```

*注意system call number不能使用 387 - 423, 也不能使用512以后的(为了兼容32位系统), 找一个没用过的就行*

#### 4 Compile and Update Kernel

编译方法详见 [wsl 更新内核](Container/wsl%20更新内核.md)

#### 5 Test

测试程序:

```c
#include <stdio.h>
#include <linux/kernel.h>
#include <sys/syscall.h>
#include <unistd.h>

int main(){
	long ret = syscall(439);
	printf("return code is: %ld\n", ret);
	return 0;
}
```

执行测试程序, 然后使用`dmesg`查看执行结果.


### 参考

> [Add a new Linux (5.7.9) system call under WSL2 (Ubuntu) - Code World](https://www.codetd.com/en/article/12278884)