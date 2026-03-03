[Redis](https://redis.io/docs/latest/develop/) 更像是一种数据结构服务器, 而不是关系型数据库; 更适合作为缓冲, 而不是存储海量数据. 实现语言是 C, 数据存储在内存中. 官方列举如下用途:
- database
- cache
- streaming engine
- message broker

Redis 的很多类型虽然类似 Python, 但是并不支持复杂的多层嵌套, 每种类型存储的都是 String. 可以借助键顺序 `a:b:c` 来实现类似嵌套字典的功能.

![why redis so fast?|600](http://oss.jay-waves.cn/til/why-redis-so-fast.png)

Redis 的 IO 模型是 [Reactor](../os/io/io-multiplexing.md) + EventLoop 

## 1. Redis 基础数据结构

### IntSet 

```cpp
typedef struct intset {
	int32_t encoding; // 16, 32, 64
	int32_t length;  
	int<T>  contents;
};
```

### SDS

Redis 字符串称为 SDS (Simple Dynamic String), 内部维护了一个内存池 (块大小为 1MB).

```cpp
struct sdshdr {
	int len;    // 已占用空间
	int alloc;  // 可用空间
	char buf[];
	int flags;
}
```

### ZipList 

元素较少时，

### quicklist 

元素较少时, 使用 `ziplist`; 元素较多时, 使用 `linkedlist`


### Stream 

Redis 5.0 后, Pub/Sub 机制默认使用 Stream 来实现.

## 2. Redis 顶层数据结构

- Strings: 表示字节序列
- Lists: 有序列表
- Sets: 无序无重复字符串集合, Redis 集合的插入/删除/访问操作为 $O(1)$ 时间.
- Hashes: 类字典的键值格式
- Sorted Sets: 有序无重复字符串集合, 每个元素有权重参数 `score` 用于排序
- Streams: 类似仅追加日志, 记录时间顺序并交付处理.
- Geospatial indexes: 空间索引
- Bitmap: 可在字符串上使用比特操作
- Bitfields: 以字节序列组织的多个计数器

| 数据结构   | 底层数据结构         |
| ---------- | -------------------- |
| String     | SDS                  |
| List       | LinkedList + ZipList |
| Hash       | HashTable + ZipList  |
| Set        | HashTable + IntSet   |
| Sorted Set (ZSet) | SkipList + ZipList                     |

### Set 

```bash
SADD k m1 m2 ...
SREM k m1 m2 ...
SPOP k [count]  # 随机弹出

SISMEMBER k m 
SMEMBERS k # 返回所有成员
SCARD k    # 返回成员个数

SINTER k1 k2 
SUNION k1 k2
SDIFF k1 k2
```

### BitMap 

```bash 
setbit k offset 0/1
bitop or k 
bitcount k

# 例子
setbit 20260301 1333 1
```

### HyperLogLog 

一种概率性结构，用极小的内存估算元素数量。主要用于可忍受一定误差的统计。

## 3. Redis 部署

### 数据过期时间

* `EXPIRE key seconds` 指定**秒**后过期
* `PEXPIRE key milliseconds` 指定**毫秒**后过期
* `EXPIREAT key temstamp` 指定**时间戳**时过期
* `TTL key` 查询剩余时间

Redis 底层**使用一个字典来保存键过期时间**。如果过期，返回 NULL，但是懒删除。定期再**随机**删除。

### 数据淘汰策略

内存达到阈值后，Redis 触发淘汰策略（maxmemory-policy）。可配置为：
* volatile-lru 从设置了过期时间的数据中，淘汰最近最少使用的数据
* volatile-ttl 从设置了过期时间的数据中，选择将要过期的数据淘汰
* volatile-lfu 从设置了过期时间的数据中，选择最不经常使用的数据
* volatile-random 从过期的数据中随机选择数据淘汰
* allkeys-lru 移除最近最少使用的数据
* no-eviction 禁止驱逐数据，内存不足时写操作直接报错


### Module 

Redis 官方支持的插件，如：
* JSON 支持
* [Bloom 过滤器支持](../algo/hash-based/bloom-filter.md)
* RedisSearch 索引引擎

### 批量操作

* 原生批量操作命令：原子的
* pipeline：将多个命令打包，非原子
