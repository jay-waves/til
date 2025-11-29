

UBI (Unsorted Block Images) 是用于裸 Flash 的文件系统管理层, 比较适合嵌入式设备. 

与 YAFFS2 和 JFFS2 相比, 它更适合大容量 NAND Flash. UBIFS 也是 [LFS](unilfs.md) 结构

## 内核模块架构

![|400](../../attach/vfs_2_flash.avif)

* MTD (memory technology device) 提供了对 Flash 裸硬件的抽象和基础接口
* UBI . 专为 NAND Flash 设计, 负责坏块管理和磨损均衡 (wear leveling). 并在 MTD 之上, 提供逻辑卷管理的功能.
* UBIFS 是运行于 UBI 上的文件系统. 

对于 U 盘 / SSD / SD 等设备, 它们通过 FTL 抽象为*块设备*, 供传统磁盘文件系统使用. 而 MTD 设备既不是字符设备, 也不是块设备. MTD 设备一般出现在 `/dev/mtdX`

## 日志型文件系统

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


