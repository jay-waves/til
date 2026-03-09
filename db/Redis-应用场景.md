Redis 设计 Key 名的规范： `Table:Column:PrimaryKey:PrimaryKeyValue`


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