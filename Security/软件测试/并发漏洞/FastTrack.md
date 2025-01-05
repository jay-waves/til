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