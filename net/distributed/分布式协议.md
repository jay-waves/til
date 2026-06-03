## CAP 理论

CAP (Consistency, Availability, Partition tolerance) 由 (Eric Brewer, 2000) 提出. 对于一个分布式系统来说, 当设计读写操作时, 只能同时满足以下三点中的两个:
* Consistency (一致性): 所有节点访问同一份最新的数据副本
* Availability (可用性): 非故障的节点在合理时间返回合理的响应
* Partition TOlerance (分区容错性): 网络出现非联通的分区时, 仍然能够对外提供服务.

一般认为, 发生网络分区故障后, 只能 Consistency 和 Avilability 中二选一进行保障. 因为常见的分布式架构只有 AP 和 CP 两种.




### 服务注册中心

* 服务注册：将自身服务信息注册到中心，暴露自身状态及访问协议
* 服务发现：向中心请求已注册的服务信息

注册中心推荐采用 AP，可用性更重要。

### 分布式锁

### 分布式事务

## Raft 协议

## 参考

https://juejin.cn/post/6844903936718012430



