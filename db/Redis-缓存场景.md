## 1. 常见缓存管理策略

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

## 2. HotKey 处理策略

#BP

Redis 缓存中某个键被频繁访问。

### 检测方法

开启 `--hotkeys` 参数，同时配置 `maxmemory-policy` 参数为 LFU 算法（剔除最不常使用键）。

### 解决方案

解决方案：
1. 读写分离：主节点处理写，从节点处理读
2. 集群：将数据分散存储在多个 Redis 节点

## 3. 分布式锁实现


## 4. Redis 能否作为消息队列？

一般不能，无法保证“至少投递一次”的语义，需要手动处理消息丢失和消息堆积。

## 5. Redis 实现延时任务处理

基于 DeleyedQueue 实现延时任务。DelayedQueue 在宕机时能够持久化，同时消息不会重复。

## 6. 设计一个排行榜

```bash 
ZADD leaderboard:game1 score user_id 
ZADD leaderboard:game1 100 userA

# 查询前10
ZREVRANGE leaderboard:game1 0 9 withscores # REVRANGE 从高到低，RANGE 是从低到高

# 查询排行
ZREVRANK leaderboard:game1 userA
ZSCORE leaderboard:game1 userA
```

## 7. 设计一个延时任务