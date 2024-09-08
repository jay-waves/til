## Linux
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


## POSIX

`/usr/include/unistd.h` 包含 Unix 系统调用标准库, 扩展 POSIX API 定义的标准系统调用. 是 POSIX 标准库的一部分, 如 `unistd.h, fcntl.h, sys/types.h`.

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


