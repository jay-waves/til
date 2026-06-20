[bloom-filter](../hash-based/bloom-filter.md)

#TODO 

## WAL 

## CheckPoint 

## Log-Structured Merge Tree 

[lsm-tree](../../attach/ascii/lsm-tree.md)

https://totoro-jam.github.io/battle-tested-patterns/zh/patterns/lsm-tree/

unilfs 是 LFS (Log-structured File System) 类型的. 数据写不会**原地覆盖旧数据**, 而是*追加 (append-only)* 到尾部. 

* LFS 顺序写的效率高, 因为无需用 inode 查表访问, 直接追加写入
* 崩溃一致性高. 用 Checkpoint 允许崩溃恢复, 因此也支持快照.
* 定期需要垃圾回收 (GC), 清除不再需要的旧版本数据. 总体的时间/空间开销仍大于传统 FS.
* 随机读取的延迟抖动大. 

### 方法

...

* Segment: 固定大小日志块. 所有新写入都顺序写到 Segments 末尾, 同时将旧版本标记为失效. 当 Segments 内部数据全部失效时, 就会被 GC 定期清除.
* Inode Map : 将 inode 映射到实际的 Segment 内部位置.
* Checkpoint: 定期落盘时, 保持一次快照. 


## 参考

* 1996 -- O'Neil -- The Log-Structured Merge-Tree (LSM-Tree)
* [2023 -- fackbook -- RocksDB Overview](https://github.com/facebook/rocksdb/wiki/RocksDB-Overview)