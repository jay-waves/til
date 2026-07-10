介绍一些

## GWP-ASan

原理类似 [KFence](kernel%20sanitizers.md), 只对部分 堆内存 操作进行采样, 替换其 `malloc(), free(), realloc()` 等操作.

https://llvm.org/docs/GwpAsan.html

## Stack Maps 

https://llvm.org/docs/StackMaps.html

## Pointer Authentication 

https://llvm.org/docs/PointerAuth.html

## MemTagSanitizer 

https://llvm.org/docs/MemTagSanitizer.html

Armv8.5-A 架构提供了 Memory Tagging Extension, 可以用于 Tag-based Sanitzier. 基本思想是, 给每块内存分配 tag, 给每个指针分配 tag, 只有 tag 匹配的指针才能读取内存. 

* memory tag: 存储在 CPU Tag Storage 中, 对应某块物理内存, 但不是内存. 通过特殊指令, 如 ARMv8 `STG` 访问.
* pointer tag: 64b 作为指令编码的只有 48b, 有部分划为 tag 区域. 

## Scudo Hardened Allocator 

每块堆内存会有一个 `header`, 包含如下信息:
* stack of the chunk: available, allocated, quarantined 
* allocation type: `malloc, new, new[], memalign`
* size 
* offset of the chunck: 从头部到实际内存区的偏移
* 16b checksum: 用来检查头部是否被污染

返回给用户的地址, 实际指向了 `header` 后的内存区域. 用户通过三种类来管理内存:
* Primary Allocator: 用于小对象分配, 速度快. 类似 Linux Slab 思想.
* Secondary Allocator: 用于大对象分配, 速度慢. 用 `mmap()` 来直接映射大块内存, 并额外添加 Guard Pages 来提供 Out-of-Bounds 隔离. 参考 GWP-ASan.
* Quaratine: 用于延迟 deallocation 操作. 有助于检测 UAF, 但是内存负担重.