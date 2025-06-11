## Redis Cli 

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
