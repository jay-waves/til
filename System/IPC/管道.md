管道 (pipe)

分类:
1. pipe, 普通管道. 单工, 单向传输, 或仅在进程组内使用.
2. s_pipe, 流管道. 半双工, 仅在进程组内使用, 双向传输.
3. name_pipe, 具名管道

![|300](../../attach/Pasted%20image%2020230619201928.png)

## 格式化管道

```c
// `cmd` is the process that will be connected to the calling process
// `type` is either "r" or "w"
FILE *popen(char *cmd, char *type);
void pclose(FILE *stream);
// then use fprintf() and fscanf() to communicate with pipe's stream
```

## 低层次管道

管道借助文件系统实现, 原理见 `fs/pipe.c`. 

使用 `pipe()` 后, 用 `fork()` 调用创建子进程, 使其通过管道和父进程通信. 通信时, 读进程关闭写端, 写进程关闭读端, 使用 `read(), write()` 进行*单向传输*, 释放管道使用 `close()`. 

```c
// returns two file descriptors, fd[0] for reading, fd[1] for writing
// return 0 on success, -1 on failure and sets errno
int pipe(int fd[2]);
```

```c
    int n;
    int fd[2];
    pid_t pid;
    char line[MAXLINE];

    if (pipe(fd) == 0) {               /* 先建立管道得到一对文件描述符 */
        exit(0);
    }

    if ((pid = fork()) == 0)          /* 父进程把文件描述符复制给子进程 */
        exit(1);
    else if (pid > 0) {               /* 父进程写 */
        close(fd[0]);                 /* 关闭读描述符 */
        write(fd[1], "\nhello world\n", 14);
    }
    else {                            /* 子进程读 */
        close(fd[1]);                 /* 关闭写端 */
        n = read(fd[0], line, MAXLINE);
        write(STDOUT_FILENO, line, n);
    }

    exit(0);
```

## 命名管道

named PIPE, 也叫 FIFO (First In, First Out). 是由内核管理的一种特殊文件, 可以用 `文件路径` 让无关的两进程建立连接.

```c
#include <sys/types.h>
#include <sys/stat.h>

int mkfifo(const char *filename, mode_t mode);
int mknode(const char *filename, mode_t mode | S_IFIFO, (dev_t) 0 );
```

实例:
```c
#include <stdio.h>  
#include <stdlib.h>  
#include <sys/types.h>  
#include <sys/stat.h>  

int main()  
{  
    int res = mkfifo("/tmp/my_fifo", 0777);  
    if (res == 0)  
    {  
        printf("FIFO created/n");  
    }  
     exit(EXIT_SUCCESS);  
}
```