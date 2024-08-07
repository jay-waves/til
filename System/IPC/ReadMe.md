进程间通信, Inter-Process Communication, IPC. 有两种基本模式:
- 共享内存: 仅建立时需系统调用, 但需并发互斥访问. 数据格式和位置自由, 速度最快, 数据量大.
- 消息传递: 使用"发送/接收消息"原语, 内核持续介入, 耗时多. 但不需避免冲突. 用于较少数据量.

![|400](../../attach/Pasted%20image%2020230619184828.png)

Unix BSD 进程通信方法:
- [管道, Pipe](Pipe.md) 与命名管道 (named pipe)
- 软中断信号 (signal), 由系统提前定义
- [套接字, Socket](Socket.md)

Unix SystemV 进程通信方法:
- 消息, Message
- [共享内存, Shared Memory](Shared%20Memory.md)
- [信号量, Semaphore](Semaphore.md)
- TLI (也是网络通信工具, 已经被淘汰)

***

- 同步通信: 进程阻塞, 直到 消息被接收/有消息可用
- 异步通信: 发送/接收消息, 并继续操作.

IPC API 有 POSIX 和 SystemV 两种接口实现, 区别见 [System V IPC vs POSIX IPC - Stack Overflow](https://stackoverflow.com/questions/4582968/system-v-ipc-vs-posix-ipc)