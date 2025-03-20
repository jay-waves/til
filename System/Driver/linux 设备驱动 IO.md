## 阻塞 IO

*阻塞操作*是指: 设备操作无法立即获得资源, 其进程被挂起, 进入睡眠状态. 
*非阻塞操作* 则并不挂起进程, 要么放弃 (返回 `-EAGAIN`), 要么不停地查询.

阻塞读取:
```c
char buf;
fd = open("/dev/ttyS1", O_RDWR);

res = read(fd, &buf, 1);
if (res == 1)
	...
```

非阻塞读取:
```c
char buf;
fd = open("/dev/ttyS1", O_RDWR | O_NONBLOCK);

while(read(fd, &buf, 1) != 1)
	continue;
...
```

除了 `open()` 调用, `ioctl()`, `fcntl()` 也可以更改读写方式, 如从阻塞变为非阻塞.

### 等待队列

## 轮询

## 异步 IO

### Linux AIO

### glibc AIO

