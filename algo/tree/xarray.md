xarray 就是老版本内核 radix tree，但是修改了接口，并保证内存安全。

```c
struct xa_node {
	uint8_t shift;  // Bits remaining in each slot
	uint8_t offset; // Slot offsert in parent 
	uint8_t count;  // total entry count 
	uint8_t nr_values; // value entry count 
	struct xa_node __rcu *parent; 
	struct xarray        *array;  // the array we belong to
	union {
		struct list_head private_list; // for tree user 
		struct rcu_head rcu_head;      // used when freeing node 
	};
	void __rcu *slots[XA_CHUNK_SIZE];
	union {
		unsigned long tags[...][];
		unsigned long marks[...][];
	};
	
}
```

`shift=0` 时，表示 `xa_node->slots` 数组中成员为叶子节点；当 `shift=n` 时，说明 `xa_node` 数组中成员指向的 `xa_node` 可以最多包含 `2^n` 个节点。

```c
struct xarray {
	spinlock_t xa_lock;
	gfp_t xa_flags;
	void __rcu* xa_head;
}
```

![xarray](../../attach/ascii/xarray.md)

`xa_node->slots` 类型是 `void**`，它能存储多种类型。要求对齐 4B，因此最低两位存储类型标签。
* Pointer Entry 。指针的最低两位是 `0b00`
* Internal Entry ，一般指向下一级 `xa_node` ，指针最低两位为 `0b10`
* Value Entry ，指针最低两位为 `0b01` 或 `0b11`
* Errno ，用负值存储。实际指向了 `0xffffffff` 附近的无效地址空间

## API 

#### 初始化 

```c
#include <linux/xarray.h>

struct xarray arr;

xa_init(&arr);

...

xa_destroy(&arr);
```


#### xa_store()

```c

void *xa_store(struct xarray *xa, unsigned long idx, void* entry, gfp_t);

// store but not override existed 
void xax_insert(struct xarray *xa, unsigned long idx, void *entry, fgp_t);
```

#### xa_erase()

```c
void *xa_erase(struct xarray *xa, unsigned long idx);
```

#### xa_load()

```c
void *xa_load(struct xarray* xa, unsigned long idx);
```

## Radix Tree vs. Trie 

Radix 树实际是一种压缩过的 Trie 树，压缩路径并按位进行分块索引。Radix 的核心参数 $r=2^{x}$ ，其中 $x$ 意为每层用 $x$ 比特来选择下一条边。

Radix 树更矮，查找性能更好，对缓存更友好。但是**每层预留的槽位更大，数据比较稀疏时，会浪费槽位空间，可能会退化为普通 Trie。**

可以发现，Radix 树在某些情景下类似页表结构，不过不限制最大深度。（页表受地址位宽范围限制）

普通字典树：
```ascii
r
|
+-- o
|   |
|   +-- m
|       |
|       +-- a
|           |
|           +-- n
|               |
|               +-- e
|               |
|               +-- u
|                   |
|                   +-- s
|
+-- u
    |
    +-- b
        |
        +-- e
            |
            +-- n
                |
                +-- s
```

Radix 树：

```ascii
root
 |
 +-- "r"
      |
      +-- "oman"
      |     |
      |     +-- "e"   -> romane
      |     |
      |     +-- "us"  -> romanus
      |
      +-- "ubens"     -> rubens
```

## Reference 

https://en.wikipedia.org/wiki/Radix_tree

http://docs.kernel.org/core-api/xarray.html