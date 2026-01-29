数据密集型程序：
* databases：存储数据
* caches：缓存中间结果
* search indexes：数据索引
* stream & batch processing：异步地、批量地 处理数据流

常见数据系统架构：

![ddis-f1-1|500](http://oss.jay-waves.cn/til/ddis-f1-1.avif)

## 网络应用的性能

从两个方面优化性能:
- throughput. 吞吐量需要测试**一个端点**处理大量消息的性能.
- response time. 
	- latency. 延迟需要测试**两个端点**传递消息的速度.
	- queueing delays. 避免*护航效应（head-of-line blocking）*。

### Batching 

ZeroMQ 将多个消息合并为一次请求处理, 避免频繁地系统调用, 从而增加系统的吞吐量. 但*批处理 (batching)* 会增加延迟, 同一批次的最先抵达的网络包, 其延迟也最大. 最佳实践是, 当消息速率超过网络带宽 (bps, msg/s...) 时, 打开网络批处理, 避免消息排队; 批处理应在整个网络栈的尽可能上层开启, 减少整体资源开销.

TCP 中的批处理算法被称为 *Nagle's Algorithm*, 通常网络场景总有消息排队 (queueing effect), 推荐开启.

![](../../attach/net-0mq-batch-processing.avif)

![](../../attach/net-zmq-batch-processing.avif)

## 数据模型

* 关系型数据模型：SQL 
* 文档型数据模型：Json，可自包含
* 对象型数据模型：Graph-based

SQL 和 NoSQL 的核心区别是：存储一个引用外部的键，还是直接存储文本信息。存储键时，信息的一致性更好，去重、修改、搜索都方便；直接存储文本信息时，存储局部性更好，对人类阅读更友好。



## 常见缓存管理策略

### Cache Aside Pattern

*旁路缓存（Cache Aside Pattern）*，指应用程序直接管理 DB 和 Cache。

应用写操作：
1. 更新 DB
2. 直接删除 Cache 中对应数据（懒删除）

应用读操作：
1. 从 Cache 读取数据
2. 若命中（Hit），直接返回
3. 若未命中（Miss），从 DB 读取数据。将读取的数据写回 Cache，然后返回。

工程上，删除 Cache 后，等待几百毫秒，再次删除一次 Cache。这样可避免并发导致的 Cache 与 DB 不一致：
* A 读，未命中缓存
* A 从 DB 读取数据，还未写回 Cache 
* B 更新 DB，删除 Cache 
* A 写回旧 Cache，和最新 DB 不一致.

### Write Through

将 Cache 视为主要存储，所有读写请求只和 Cache 交互。DB 对应用程序透明。

该模式并不常见。因为每次写操作都要同时更新 DB + Cache。

### Write Back

将 Cache 视为主要存储。Write Through 会同时更新 Cache + DB；而 Write Back 只更新 Cache，异步地批量更新 DB。

Write_Back 应用场景有限，因为如果 Cache 挂了就可能丢失数据。

## 参考

*Designing Data-Intensive Applications*. Martin Kleppmann. 2017. 1e.

https://github.com/ByteByteGoHq/system-design-101

[The Architecture of Open Source Applications](https://aosabook.org/en/index.html).

https://aosabook.org/en/v2/zeromq.html

https://aosabook.org/en/posa/high-performance-networking-in-chrome.html

Open-MX-IOAT.pdf

https://aosabook.org/en/posa/secrets-of-mobile-network-performance.html

https://aosabook.org/en/posa/warp.html
