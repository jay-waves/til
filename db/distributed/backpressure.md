#TODO 

## 限流器（Rate Limiter）

限流器最常见的设计是*令牌桶 (Token Bucket)*  。系统按固定速率往桶里放 token ，网络请求先拿 token，没有 token 就拒绝或排队。通过空闲时的 token 累计来允许短时间**突发流量（burst）**，但是又能用 (refill rate, req/s) 限制长期平均速率。

需要注意，限流器无法精确控制并发数，只能控制请求的通行速率。如果任务耗时长（较重），并发数还是会堆积。

```
refill:
  tokens += elapsed * rate
  clamp to burst

consume:
  if tokens >= n:
     tokens -= n
  else:
     wait = missing / rate
```

主要有两种实现方式：
* On-demand Token Bucket：不周期性补 Token（没有后台定时器），只有当请求到来时，才根据 now - last_refill 补充 Token，最多补充某个上限（burst limit）。空闲开销低。
* Periodic Refill: 有周期性定时器，但是周期过小浪费资源，周期过大会增加发送抖动。只适合需要明确节拍控制发送周期，而不是只需要平均速率控制。

## 熔断器（Circuit Breaker）

在打开/关闭中间加入一个*半开*状态，用少量请求验证服务的恢复情况，同时限制风险。

```ascii
        +------+
        | close|
        +------+
           |
           | failure rate exceeds threshold
           v
        +------+
        | open |
        +------+
           |
           | cooldown period expires
           v
     +-----------+
     | half_open |
     +-----------+
        |      |
        |      |
        |      +--------------------+
        |                           |
        | probe succeed             | probe fail
        v                           v
     +------+                    +------+
     | close|                    | open |
     +------+                    +------+
```

* close: 开关闭合，允许流量全量通过
* open: 开关断开，拒绝流量通过，快速失败
* half-open: 开关半开，允许少量真实流量通过，测试下游服务情况


## 指数退避重试（Retry with Backoff）

不立即重试（会压垮故障服务），也不直接放弃，而是每次重试加倍等待时间。加抖动让成千上万客户端不同时重试。避免惊群效应。

```
  Time ────────────────────────────────────────────────►

  Attempt 1  ✗ ├─┤ 1s
  Attempt 2  ✗ ├───┤ 2s
  Attempt 3  ✗ ├───────┤ 4s
  Attempt 4  ✗ ├───────────────┤ 8s
  Attempt 5  ✗ ├───────────────────────────────┤ 16s (cap)
  Attempt 6  ✓

  Each bar = wait before next retry (doubles each time)
  + jitter: randomize within each bar to avoid thundering herd
    
1s → 2s → 4s → 8s → 16s cap 

Each bar represents the wait before the next retry.
The wait duration doubles after each failed attempt until it reaches the cap.
Jitter randomizes the actual wait time within each bar to avoid a thundering herd.
```

## 一致性哈希

详见 [consistent-hash](../../algo/hash-based/consistent-hash.md) 和 [bloom-filter](../../algo/hash-based/bloom-filter.md)

## 参考

[Battle-Tested Patterns](https://totoro-jam.github.io/battle-tested-patterns)