## 网络编程

IO 模型:
- asio 
- zmq

并发模型: (见 C/C++)
- libuv, libevent 
- libcoro 
- libexecution

### Reactor 

Reactor 模型中, **应用主动响应 IO 事件**.

1. Reactor 后台运行, 检测到某个 IO 事件就绪
2. 将事件通知给应用
3. 应用层自行执行具体 IO 动作

### Proactor 

Proactor 模型中, **应用提交操作, 操作完成后被动接受结果**. 

1. 应用将动作 (read/write) 提交给 Proactor 
2. Proactor后台执行, 并自动完成 IO 动作
3. Proactor 将动作结果直接通知应用.

|              | Actor    |
| ------------ | -------- |
| libevent     | Reactor  |
| Netty (Java) | Reactor  |
| Windows IOCP         | Proactor |
| Linux epoll  | Reactor  |
| Linux io_uring | Proactor |
| POSIX AIO    | Proactor |
| ASIO         | Proactor |

## 网络编程性能优化

从两个方面优化性能:
- throughput. 吞吐量需要测试**一个端点**处理大量消息的性能.
- latency. 延迟需要测试**两个端点**传递消息的速度.

### Batching 

ZeroMQ 将多个消息合并为一次请求处理, 避免频繁地系统调用, 从而增加系统的吞吐量. 但*批处理 (batching)* 会增加延迟, 同一批次的最先抵达的网络包, 其延迟也最大. 最佳实践是, 当消息速率超过网络带宽 (bps, msg/s...) 时, 打开网络批处理, 避免消息排队; 批处理应在整个网络栈的尽可能上层开启, 减少整体资源开销.

TCP 中的批处理算法被称为 *Nagle's Algorithm*, 通常网络场景总有消息排队 (queueing effect), 推荐开启.

![](../../../attach/Pasted%20image%2020250503221227.avif)

![](../../../attach/Pasted%20image%2020250503221234.avif)

### 

https://aosabook.org/en/posa/high-performance-networking-in-chrome.html

D:\desktop\Open-MX-IOAT.pdf

https://aosabook.org/en/posa/secrets-of-mobile-network-performance.html

https://aosabook.org/en/posa/warp.html

## 参考

https://aosabook.org/en/v2/zeromq.html