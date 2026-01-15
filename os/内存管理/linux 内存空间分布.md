
## 虚拟内存分布

| 区段         | 作用 | 64 位起止地址 | 32 位起止地址 | 可见性       |
| ------------  | ---- | ------------- | ------------- | ------------ |
| kernel space   |      |  0xffff8000 00000000 - 0xffffffff ffffffff         |  0xc0000000 - 0xffffffff            | 所有进程共享 |
| user space     |      | 0x00000000 00400000 - 0x00007fff fffff000             | 0x08048000 - 0xc0000000              |  进程间独立           |
| reserved user space       | 空指针错误     |   0x00000000 00000000 - 0x00000000 00400000            |  0x00000000 - 0x08048000             |   进程间独立           |

注意到 64 位下, `0x00007fff fffff000 - 0xffff8000 00000000` 这一段内存并没有使用
, 被称为内存空洞 (canonical address hole). 实际只使用了 48 位地址, 内核态和用户态各 128TB, 分布在逻辑内存最高处和最低处.

linux 下, 一个进程被称为 task, 所有 task 共享一个内核态虚拟空间, 而拥有独立的用户态
虚拟内存空间, 因此这部分空间也被称为 `TASK_SIZE`. 

```c
// in arch/x86/include/asm/page_32_types.h
#define TASK_SIZE       __PAGE_OFFSET

/*
 * task_size 代表用户空间进程内存大小
 * 32 位系统, task_size 为 0xc000 0000, 大小为 3GB
 * 64 位系统, task_size 为 0x0000 7ffff ffff f000, 大小为 128TB
*/
```


## 用户态空间分布

用户态虚拟内存分布 (地址从高到底):

| 区域                 | 英文名                  | 起始标识         | 结束标识 | 描述                                                        |
| -------------------- | ------------------ | ---------------- | -------- | ----------------------------------------------------------- |
| 栈                   | stack                         | `start_stack`    |          | 函数调用和函数本地变量信息, 从高地址 (start_stack) 向下增长 |
| 内存映射与匿名映射区 | memory-mapped file            | `mmap_base`      |          | 用于共享内存或存储依赖的动态库, 向下增长. 文件, 开辟大内存和共享库都映射到独立的 VMA.                    |
| 堆                   | heap                |`start_brk`  | `brk` (当前堆顶)          | 用户动态开辟小内存 (<128K), 向上增长                        |
| BSS段                | bss                |`end_data`   | `start_brk`               | 存放未初始化的静态或全局变量, 默认全部设置为0               |
| 数据段               | data               |`start_data` | `end_data`                | 初始化的静态或全局变量                                      |
| (内存屏障)           | barrier                         |                  |          | 不可访问段, 用于预防栈溢出威胁可执行代码段                  |
| 代码段               | code               |`start_code` | `end_code`                         | 可执行程序代码                                              |
| 保留区域             | reserved                       |                  |          | 预防空指针, 惯例保留不用                                    |

32 位架构下, 栈大小默认为 8MB.

在 `/proc` 下可查看进程的虚拟空间分布. 当设备号和节点都为0时, 标识其是临时的 (不映射到文件中), 称为匿名虚拟内存区域 (Anonymous VMA). 
```sh
cat /proc/12/maps
[sudo] password for JayWaves:
# VMA 地址范围     权限  段偏移   设备号  节点号                 映射文件路径  
00200000-00277000 r--p 00000000 00:02 5                         /init
00277000-003c3000 r-xp 00076000 00:02 5                         /init
003c3000-003d8000 rw-p 001c1000 00:02 5                         /init
003d8000-003da000 rw-p 001d5000 00:02 5                         /init
003da000-003e1000 rw-p 00000000 00:00 0
00e4f000-00e50000 ---p 00000000 00:00 0                         [heap]
00e50000-00e51000 rw-p 00000000 00:00 0                         [heap]
7fe19d7c6000-7fe19d7ca000 rw-p 00000000 00:00 0
7fe19d7ca000-7fe19d7cc000 rw-p 00000000 00:00 0
7fe19d7cc000-7fe19d7ce000 ---p 00000000 00:00 0
7fe19d7ce000-7fe19d7fc000 rw-p 00000000 00:00 0
7ffcaf2ab000-7ffcaf2cc000 rw-p 00000000 00:00 0                 [stack]
7ffcaf3f5000-7ffcaf3f9000 r--p 00000000 00:00 0                 [vvar]
7ffcaf3f9000-7ffcaf3fb000 r-xp 00000000 00:00 0                 [vdso] 
```

## 内核态空间分布

### 32 位内核虚拟地址空间

|            | 起始            | 结束                      | 描述  |
| ---------- | --------------- | ------------------------- | ----- |
| 临时映射区 |                 | 0xffff ffff               |  用于临时映射, 缓存等     |
| 固定映射区 | `FIXADDR_START` | `FIXADDR_TOP` = 0xffff f000 | 虚拟内存地址固定为特定用途, 映射的物理地址可变.      |
| 永久映射区 | `PKMAP_BASE`    | `FIXADDR_START`           |  允许虚拟内核内存与物理内存建立长期映射关系.     |
| 动态映射区 | `VMALLOC_START` | `VMALLOC_END`             |  `vmalloc()` 动态映射到高地址物理内存区域, 大小为2页     |
| 空洞       | 0xf800 0000     | `VMALLOC_START`           | 8MB   |
| 直接映射区 | 0xc000 0000     | `high_memory` = 0xf800 0000               | 896MB |

物理空间分布如下 (地址从高到低). 在 64b 系统上, `ZONE_HIGHMEM` 已经无用.

| 区域名称     | 起止地址                  | 大小  |
| ------------ | ------------------------- | ----- |
| ZONE_HIGHMEM | 0x3800 0000 -             |       |
| ZONE_NORMAL  | 0x0100 0000 - 0x3800 0000 | 880MB |
| ZONE_DMA     | 0x0000 0000 - 0x0100 0000 | 16MB  |

### 64 位内核虚拟地址空间

x86-64 实际只支持 48b 虚拟地址, 用户态和内核态各占 128TB, 高 16b 自动符号扩展 (canonical address). 
- 有效用户态地址范围: `0x0000_0000_0000_0000` --> `0x0000_7fff_ffff_ffff`
- 有效内核态地址范围: `0xffff_8000_0000_0000` --> `0xffff_ffff_ffff_ffff`

AMD64标准规定, 48b 地址的最高位需要符号扩展, 使其与 64b 的高 16b 保持一致. 这样才能在直接忽略高 16b 地址的同时, 区分 `0xffff_...` 和 `0x0000_...`. 因此地址实际的有效位只有 47b.

```
------------------------------
0xffffffffffffffff 
	
    Reserved
    
0xffff_ffff_a000_0000
------------------------------
	Kernel .text / .rodata 
	Kernel .data / .bss
	
0xffff_ffff_8000_0000 = __START_KERNEL_MAP
------------------------------
	Per-CPU data 
------------------------------
	Kernel Stacks (Per-thread)
------------------------------
	SLAB / SLUB caches 
------------------------------

	Page Tables (vmemmap --> struct page[], 1TB)
	
0xffff_ea00_0000_0000 = VMEMMAP_START
------------------------------
	Buddy Aloocator Pool 
------------------------------
	Page Cache (file-backed)
------------------------------
	Module Space 
------------------------------
0xffff_e900_0000_0000
	
	vmalloc area (Non-contiguous Kernel heap)
------------------------------
	ioremap region (Device MMIO remapped)

0xffff_c900_0000_0000
------------------------------
	fixmap region 
------------------------------
0xffff_8800_0000_0000 = PAGE_OFFSET
	
	Linear RAM Mapping (64TB)
	
0xffff_c800_0000_0000
------------------------------
0xffff_8000_0000_0000
------------------------------
0xffff_7fff_ffff_ffff
	
	Canonical Address
	
0x0000_8000_0000_0000
------------------------------
0x0000_7fff_ffff_ffff = TASK_SIZE_MAX
	
	User Space (Per-process Virtual, 128TB)
	
0x0000_0000_0000_0000
------------------------------
```

### 直接映射区

32 位架构中, 内核态前 896MB 空间被称为直接映射区.  虽然仍是虚拟内存, 但整块内存被
**直接一一映射**到**物理内存**的前 896MB 上. 该空间也被分为不同区域:

- 前 1MB 载入系统执行文件 
- 前 16MB 页框用于 DMA 设备控制器寻址, 该区域称为 `DMA_ZONE`.
- 16-896MB 用于存储正常页框, 称为 `NORMAL_ZONE`. 

区域中主要存储用户进程相关的内容:
- 系统进程的核心数据结构 `task_struct, mm_struct, vm_area_struact` 等.
- 内核栈: 每个进程的内核栈默认为2页, 存放进程的调用链.
