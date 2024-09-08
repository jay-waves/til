[Redis](https://redis.io/docs/latest/develop/) 更像是一种数据结构服务器, 而不是关系型数据库; 更适合作为缓冲, 而不是存储海量数据. 实现语言是 C, 数据存储在内存中. 官方列举如下用途:
- database
- cache
- streaming engine
- message broker

Redis 的很多类型虽然类似 Python, 但是并不支持复杂的多层嵌套, 每种类型存储的都是 String. 可以借助键顺序 `a:b:c` 来实现类似嵌套字典的功能.

### Redis DataStructure

- Strings: 表示字节序列
- Lists: 有序列表
- Sets: 无序无重复字符串集合, Redis 集合的插入/删除/访问操作为 $O(1)$ 时间.
- Hashes: 类字典的键值格式
- Sorted sets: 有序无重复字符串集合
- Streams: 类似仅追加日志, 记录时间顺序并交付处理.
- Geospatial indexes: 空间索引
- Bitmap: 可在字符串上使用比特操作
- Bitfields: 以字节序列组织的多个计数器

### Redis Cli 

connect to redis server :
```bash
redis-cli -h 127.0.0.1 -p 6379

redis-cli flushdb

redis-cli Keys '*'

# 获取所有键值
redis-cli KEYS '*' | xargs -n 1 redis-cli GET

redis-cli Info keyspace
```

### Others

`Watch` 监控 mykey 状态, 如果 `Wacth` 期间键被修改, 后续 `EXEC` 会直接退出.

```bash
# 使用 watch 实现 ZPop
WATCH zset
element = ZRANGE zset 0 0
MULTI
ZREM zset element
EXEC
```

Redis `Pub/Sub` 实现了一种发布/订阅通信机制. 该机制和 Redis 存储机制是完全隔离的.

```bash
> Subscribe channel1 channel2
> Publish channel1 "hello, boy!"
```

## config

`sudo vim /etc/redis/redis.conf`

开启 json 模块:

1. 先编译模块:
```bash
git clone --recursive https://github.com/RedisJSON/RedisJSON.git
cd RedisJSON
./sbin/setup
make build
```

2. 配置模块:
```conf
loadmodule /usr/lib/redis/modules/rejson.so
```