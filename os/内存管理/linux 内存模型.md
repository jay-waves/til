
linux 内存管理的主要作用是隔离多个进程的内存区域, 实现虚拟内存到实际物理内存的转换. 包括 物理内存管理 / 虚拟内存管理 / 内存映射 / 接口 四个层次.

在*非均匀内存访问 (NUMA, Non-Uniform Memory Access,一般指多路 CPU 结构)* 系统中, 物理内存被划分为*节点 (Node)*, 每个节点又划分为*区域 (Zone)*, Zone 的种类代表了内存分配的策略. 每个 Zone 都有独立的*伙伴系统 (Buddy System)*, 用于分配物理*页 (Page)*. 

- `ZONE_DMA`: 低端内存, 支持老设备 DMA.
- `ZONE_NORMAL` 4GB 以内.

虚拟内存*页 (Page)* 则由*页面分配器 (Allocator)* 管理. 用*虚拟内存区域 (VMA)* 管理虚拟内存段 (如 .data, .stack, 一段连续的内存页), VMA 已经被映射为物理内存, 可能代表磁盘上文件的一部分, 也可能是临时内存 (如 heap, stack, 被称为 *anonymous page*)

虚拟内存地址和物理内存地址通过*页表 (Page Table)* 映射.

## 内存结构

### 物理内存管理

EFI/BIOS 在加载时会告知内核物理内存的分布情况, 在 `start_kernel()` 中划分内存 Zone, 将 `struct page[]` 中的区域分配给 Zone.

```c
// include/linux/mm_types.h
struct page {
  unsigned long flags; // locked, uptodate, reclaim, mlocked
  atomic_t _refcount; // -1 then page is free
  atomic_t _mapcount;
  unsigned long private;
  struct address_space *mapping; // 内存空间, file-backed/anonymous
  pgoff_t index; 
  struct list_head lru;
  void *virtual; // pages's virtual address, NULL for high memory
};

/*
	每个物理页框均有 struct page, 存放在全局数组中.
	访问页框时, 使用*物理页号 (Page Frame Number, PFN)* 索引 mem_map[PFN]
*/
struct page mem_map[];

struct zone {
  unsigned long watermark[NR_WMARK]; /* 水位线, 控制内存分配压力 */

  spinlock_t lock;
  struct free_area free_area[MAX_ORDER]; // used by buddy system 
  
  spinlock_t lru_lock;
  struct zone_lru {
	  struct list_head list;
	  unsigned long nr_saved_scan;
  } lru[NR_LRU_LISTS];
  
  struct zone_reclaim_stat reclaim_stat;
  unsigned long pages_scanned; 
  
  unsigned long flags; /* dirty? */
  
  atomic_long_t vm_stat[NR_VM_ZONE_STAT_ITEMS];

 /* wait table .... */
  
  struct pglist_data *zone_pgdat; // 所属 Node
  unsigned long zone_start_pfn; // 起始 PFN,
  unsigned long spanned_pages;  // 覆盖的 PFN 区间
  unsigned long present_pages;  // 实际存在的物理页
};

// linux5.16, 用于解决 page 过碎的问题, 表达多个连续的物理页对象
struct folio {
	struct page *page; // 指向第一页 struct page 
	unsigned int order; // buddy system 使用 2^orderd 页
	unsigned long nr_pages; // 实际包含多少个物理页
	/*
		其他字段类似 struct page 
	*/
};
```

### 虚拟内存管理

声明虚拟内存时, 不会实际映射到物理内存. 仅当使用虚拟内存时, 通过触发*[缺页中断 (Page Fault)](虚拟内存.md)*, 才会完成映射. 

```c
page fault 
	--> handle_mm_fault()
		--> alloc_pages() 
		--> vma flags: rwx, shared, anonym, file-backed, vm_page_prot
```

```c
/*
	VMA, Virtual Memory Area. 指内存一块连续区域
*/
struct vm_area_struct {
	struct file *file; // 若映射文件

	struct mm_struct *vm_mm; // 所属的地址空间 mm_struct
	unsigned long start, end;  //vm_mm 的起止
	
	struct vm_area_struct *next, *prev; // VMA 链表, 已排序
	struct rb_node vm_rb; // 红黑树, 用于快速索引.

	// Access Permissions 
	pgprot_t vm_page_prot;
	unsigned long vm_flags; // 比如 VM_READ & VM_WRITE, VM_EXEC
	unsigned long vm_offset;

	struct inode *vm_inode; 
	struct file  *vm_file; // 可能映射到的文件
	unsigned long vm_pgoff // vm_file 中的页内偏移 (PAGE_SIZE)

	const struct vm_operations_struct {
		void (*open)(struct vm_area_struct *area);
		void (*close)(struct vm_area_struct *area);
		...
	} *vm_ops;
};

/*
	include/linux/mm_types.h
*/
struct mm_struct {
	struct vm_area_struct *mmap;  // VMA 链表头
	struct rb_root        *mm_rb; // VMA 红黑树 (用于快速索引)

	int            count;
	pgd_t         *pgd;                  // 物理页表
	unsigned long context;
	unsigned long task_size; 

	unsigned long start_code, end_code; // code
	unsigned long start_data, end_data; // data 
	unsigned long start_brk, brk;       // heap 
	unsigned long start_stack;          // stack 

	// pages count 
	unsigned long total_vm;  // total pages mapped
	unsigned long locked_vm; // pages cnanot swapped out 
	...
};
```

### 页映射

Linux x64 使用四级页表来实现物理 / 虚拟内存地址映射. 这个过程中, Kernel 负责创建和维护页表项, 而虚拟地址与物理地址的转换 (查表并翻译) 则通过硬件结构 MMU (+TLB) 实现.

```
pgd_alloc()
	--> p4d (page global directory)
		--> pud (upper)
			--> pmd (middle)
				--> pte (page table entry)
```

```c
// page table entry
typedef struct {
	uint64_t present    : 1; // 是否在内存中
	uint64_t rw         : 1; // 0 只读, 1 可读写
	uint64_t us         : 1; // 0 标识内核态可访问, 1 标识用户态也可以访问.
	uint64_t pwt        : 1; // 页级写透缓存策略
	uint64_t pcd        : 1; // 页级缓存禁用, 1 标识禁用缓存
	uint64_t accessed:  : 1; // 1 标识被访问过
	uint64_t dirty      : 1; // 1 标识页已被写过 
	uint64_t pat        : 1; // 页属性表
	uint64_t global     : 1; // 全局位, 1 标识全局页, 不刷新 TLB
	uint64_t availabel  : 3; // 保留给操作系统软件使用
	uint64_t pfn        : 40; // **物理页框号**, 12-51 位
	uint64_t reserved   : 11; // 保留, 置 0 填充
	uint64_t nx         : 1; // 1 页面不可执行
} pte_t[];
```

操作系统通过下列掩码检查和设置对应权限:

| 标志位宏          |                                                        |
| ------------- | ------------------------------------------------------ |
| `V`           | 此 PTE 有效                                            |
| `FOE`         | Fault on Execute, 尝试执行该页面时, 处理器将执行缺页中断 |
| `FOW`         | Fault on Write, 尝试写入该页面时, 处理器将执行缺页中断   |
| `FOR`         | Fault on Read, 尝试读取该页面时, 处理器将执行缺页中断                                          |
| `KRE`, `KWE`  | 内核态程序可以读/写该页面.                             |
| `URE`, `UWE`  | 用户态程序可以读/写该页面.                             |
| `_PAGE_DIRTY` |                                                        |
| `_PAGE_ACCESSED`              |                                                        |


### 页缓存及置换

内存回收路径 (Reclaim)

## 内存接口

- `kmalloc(GFP_*)` 开辟内核内存, 单次申请内存需小于 `PAGE_SIZE`. 连续的小内核对象, 会被交给 SLAB/SLUB 缓存托管.
- `kcalloc()` 
- `vmalloc()` 开辟一块连续虚拟地址空间, 但不保证物理地址空间连续
- `vmap(), vunmap()` 将一段内存页映射到连续虚拟地址空间.
- `slab, slub, slob caches`: 内核的对象缓存池, 较新的是 **SLUB**.
- `dma_alloc_coherent()`
- `alloc_pages(order)` 申请连续物理页框. 使用 Buddy
- `get_free_page()` legacy 
- `mmap()` 用户空间内存. POSIX 接口.
- `brk()` 扩展堆. POSIX 接口.
- `ioremap()`: 将设备物理内存映射到内核空间

### alloc_pages

开辟 `2^order` 连续内存页, 并返回指向第一个页的指针.

```c
struct page * alloc_pages(gfp_t gfp_mask, unsigned int order);
// allocate single page
struct page* alloc_pages(gfp_t gfp_mask);
// return a page filled with zeros. used before pages were given to 
// user space, because previous memory may've contained sensitive info.
unsigned long get_zeroed_page(unsigned int gfp_mask);
void free_pages(unsigned long addr, unsigned int order);
```

### GFP mask

GFP (get free page) mask flags, `gfp_t`, 用于
- 规定内核开辟内存的行为
- 规定内存区域的类型 (如 DMA, normal)

| Action Flags    |                                                                                                                                                                                                                                            |
| --------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `__GFP_WAIT`    | allocator can sleep                                                                                                                                                                                                                        |
| `__GFP_IO`      | allocator can start disk I/O                                                                                                                                                                                                               |
| `__GFP_FS`      | allocator can start filesystem I/O                                                                                                                                                                                                                                                                                                                                                                                                      |
| `__GFP_NOFAIL`  | allocator repeats the allocation until it succeeds                                                                                                                                                                                         |
| `__GFP_NORETRY` | allocator never retries if allocation fails                                                                                                                                                                                                |
| Zone Flags      |                                                                                                                                                                                                                                            |
| `__GFP_DMA`     | allocates only from `ZONE_DMA`                                                                                                                                                                                                                                                                                                                                                                                                                      |
| `__GFP_HIGHMEM` | allocates from `ZONE_HIGHMEM` or `ZONE_NORMAL`                                                                                                                                                                                             |
| Type Flags      | Zone + Action                                                                                                                                                                                                                                           |
| `GFP_ATOMIC`    | high priority and must not sleep. used in interrupt heandlers, in bottom halves                                                                                                                                                            |
| `GFP_NOIO`      | This allocation can block, but must not initiate disk I/O. This is the flag to use in block I/O code when you cannot cause more disk I/O, which might lead to some unpleasant recursion.                                                                                                                                                                                                                                           |
| `GFP_NOWAIT`    | Like `GFP_ATOMIC`, except that the call will not fallback on emergency memory pools. This increases the liklihood of the memory allocation failing.                                                                                                                                                                                                                                           |
| `GFP_KERNEL`      | This is a normal allocation and might block. This is the flag to use in process context code when it is safe to sleep. The kernel will do whatever it has to do to obtain the memory requested by the caller. Should be **default choice** |
| `GFP_USER`      | This is a normal allocation and might block. This flag is used to allocate memory for user space processes.                                                                                                                                |
| `GFP_DMA`       | This is an allocation from `ZONE_DMA`. Device drivers that need DMA-able memory use this flag, usually in combination with one of the preceding flags.                                                                                     |

```c
	ptr = kmalloc(size, __GFP_WAIT | __GFP_IO | __GFP_FS);
```
