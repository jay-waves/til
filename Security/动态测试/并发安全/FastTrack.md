---
tags: [Paper, ]
source: https://doi.org/10.1145/1543135.1542490
copyright: [Cormac Flanagan (2009), Stephen N. Freund (2009) ]
---

## FastTrack: efficient and precise dynamic race detection

- [Eraser](Eraser.md): low cost, low precision
- [Happens Before](Happens%20Before.md): high cost, high precision
- FastTrack: low cost, high precision

FastTrack: Efficient and Precise Dynamic Race Detection. 是对 [Happens Before (HB)](Happens%20Before.md) 算法的高效实现, 仍是基于逻辑时钟 (Vector Clock 在 FastTrack 中被称为 Epoch). 
- sound & complete: 尽快地在每个变量上发现至少一个数据竞争.
- effcient
- Insight:
	1. Happens-Before 是一种偏序关系.
	2. 但对变量的访问几乎全是有序的.

### Algorithm

为每个线程 `T` 维护一个整型时钟 `c[T]`, `Epoch=(T, c[T])` 来标识逻辑时刻.

对于每个共享内存变量 `X`, 维护其最后一次写入的 Epoch, 以及读者合集 readerEpochs. 只在必要时刻使用完整 vector clock 操作.

### 是否并发?

如何判断两个 Epoch 是否并发? 并发, 当且仅当, 不存在发生序关系. 

对于 `E1 = id1@t1` 和 `E2 = id2@t2`, 满足以下任一条件, 称为 E1 happens before E2 (`E1 -> E2`)
1. t1 < t2, 且 E1 执行在 E2 开始前完成.
2. id1 的优先级高于 id2, 且 e1 在 e2 之前触发.
3. E1 发生在中断被禁用的时间内, E2 发生在中断之后.

如果 `E1->E2` 和 `E2->E1` 都不成立, 两个 Epoch 就是并发的.

### 是否竞争?

如果两个 Epoch 是并发的, 需要进一步判断两者是否存在竞争. 竞争的必要条件是
两个并发事件访问同一共享资源, 至少一个是写操作, 且没有同步机制保护.

对于 E1 和 E2, 检查如下条件:
1. E1 和 E2 访问同一资源 (内存地址, 寄存器)
2. 访问类型冲突:
	- E1 写, E2 写
	- E1 写, E2 读
	- E1 读, E2 写
3. 没有锁 / 中断禁用 等机制来同步.