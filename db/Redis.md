[Redis](https://redis.io/docs/latest/develop/) 更像是一种数据结构服务器, 而不是关系型数据库; 更适合作为缓冲, 而不是存储海量数据. 实现语言是 C, 数据存储在内存中. 官方列举如下用途:
- database
- cache
- streaming engine
- message broker

Redis 的很多类型虽然类似 Python, 但是并不支持复杂的多层嵌套, 每种类型存储的都是 String. 可以借助键顺序 `a:b:c` 来实现类似嵌套字典的功能.

![why redis so fast?|600](http://oss.jay-waves.cn/til/why-redis-so-fast.png)

## Redis DataStructure

- Strings: 表示字节序列
- Lists: 有序列表
- Sets: 无序无重复字符串集合, Redis 集合的插入/删除/访问操作为 $O(1)$ 时间.
- Hashes: 类字典的键值格式
- Sorted sets: 有序无重复字符串集合
- Streams: 类似仅追加日志, 记录时间顺序并交付处理.
- Geospatial indexes: 空间索引
- Bitmap: 可在字符串上使用比特操作
- Bitfields: 以字节序列组织的多个计数器

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
	int len;  // 已占用空间
	int free; 
	char buf[];
}
```

### ZipList 



### quicklist 

元素较少时, 使用 `ziplist`; 元素较多时, 使用 `linkedlist`


### Stream 

Redis 5.0 后, Pub/Sub 机制默认使用 Stream 来实现.

## 常见缓存管理策略

### **Cache Aside Pattern, 旁路缓存**

应用程序直接管理 DB 和 Cache.

应用写操作:
1. 更新 DB
2. 直接删除 Cache 中对应数据. (懒删除)

应用读操作:
1. 从 Cache 读取数据
2. 命中 (Hit), 直接返回
3. 未命中 (Miss), 从 DB 读取数据. 将读取的数据写回 Cache, 然后返回.

工程上, 删除 Cache 后, 等待几百毫秒, 再次删除一次 Cache. 避免并发导致的 Cache 与 DB 不一致:
* A 读, 未命中缓存
* A 从 DB 读取数据, 还未写回 Cache 
* B 更新 DB, 删除 Cache 
* A 写回旧 Cache, 和最新 DB 不一致.

### Write Through

将 Cache 视为主要存储, 所有读写请求只和 Cache 交互. DB 对应用程序透明. 

该模式并不常见. 因为每次写操作都要同时更新 DB + Cache.

### Write Back

将 Cache 视为主要存储. Write Through 会同时更新 Cache + DB, 而 Write Back 只更新 Cache, 异步地批量更新 DB.

Write_Back 应用场景有限, 因为如果 Cache 挂了就可能丢失数据.