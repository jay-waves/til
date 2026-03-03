一般文件读取流程：磁盘 --> 内核页缓存 --> 用户缓冲区

## mmap 

```c
mmap(...)
```

用户空间直接映射 PageCache，不再需要从内核内存拷贝到用户内存。

## splice 

允许数据直接在内核中流动，不经过用户空间。比如从 fd 直接发送到 socket

## sendfile 

类似 splice，减少用户态中转



## io_uring 

见 [linux-io_uring](../io/linux-io_uring.md)