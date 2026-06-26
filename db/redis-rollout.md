Redis 设计 Key 名的规范： `Table:Column:PrimaryKey:PrimaryKeyValue`


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
```## 6. 设计一个排行榜

```bash 
ZADD leaderboard:game1 score user_id 
ZADD leaderboard:game1 100 userA

# 查询前10
ZREVRANGE leaderboard:game1 0 9 withscores # REVRANGE 从高到低，RANGE 是从低到高

# 查询排行
ZREVRANK leaderboard:game1 userA
ZSCORE leaderboard:game1 userA
```