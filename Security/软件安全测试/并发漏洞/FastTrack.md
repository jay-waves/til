---
tags: [Paper, ]
source: https://doi.org/10.1145/1543135.1542490
copyright: [Cormac Flanagan (2009), Stephen N. Freund (2009) ]
---

## FastTrack: efficient and precise dynamic race detection

- [Eraser](../并发漏洞/Eraser.md): low cost, low precision
- [Happens Before](Happens%20Before.md): high cost, high precision
- FastTrack: low cost, high precision

FastTrack: Efficient and Precise Dynamic Race Detection
- sound & complete: 尽快地在每个变量上发现至少一个数据竞争.
- effcient
- Insight:
	1. Happens-Before 是一种偏序关系.
	2. 但对变量的访问几乎全是有序的.