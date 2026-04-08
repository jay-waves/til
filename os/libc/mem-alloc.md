关注点：
1. 内存安全性
2. 高并发场景下，锁竞争或缓存行争用
3. 内存碎片问题

## mmap 

`mmap` 建立一个虚拟内存段（VMA），映射到文件或匿名页，但不会立即分配物理内存。直到第一次访问该内存时，才会触发缺页中断。分配粒度为页（4KB）

```c
void* p = mmap(NULL, size, PROT_READ|PROT_WRITE,
               MAP_PRIVATE|MAP_ANONYMOUS, -1, 0);
```

* `MAP_SHARED` 写时，回写到文件
* `MAP_PROVATE` 写时，触发 COW

一般而言， `mmap()` 得到的内存，内核保证是零填充页

## libc 

* `void* malloc(n * sizeof())` 垃圾值内存
* `free(void*)`
* `void* calloc(n, sizeof())` 初始化为零
* `void* realloc(void* p, size_t new_sz)` 调整指针指向的内存大小，同时保留数据（参考 new_sz，缩容时可能丢弃部分原数据）。不保证原地扩容，可能拷贝数据搬迁，**旧指针可能失效**。

## glibc malloc

小对象直接堆分配（用 `brk()`），大对象用 `mmap()` 映射。


#### 简易实现

每个内存块抽象为 `chunk_t` ，内存块前有一些元数据，在 `k_chunk_overhead` 后是给用户使用的内存数据。

```c
enum {
    FLAG_IN_USE  = 1u << 0,
    FLAG_IS_MMAP = 1u << 1,
};

static const size_t k_align = sizeof(max_align_t);
static const size_t k_heap_grow = 64 * 1024;
static const size_t k_mmap_threshold = 128 * 1024;
static const size_t k_min_split = 32;

// 向上取整，注意 k_align 是 power_of_2
static size_t align_up(size_t n) {
    return (n + k_align - 1) & ~(k_align - 1);
}

typedef struct chunk_t {
    size_t size;       
    uint32_t flags;

    struct chunk_t* prev_phys;
    struct chunk_t* next_phys;

    struct chunk_t* prev_free;
    struct chunk_t* next_free;
} chunk_t;

// 向上取整
static const size_t k_chunk_overhead =
    ((sizeof(chunk_t) + k_align - 1) / k_align) * k_align;
    
static void* chunk_to_user(chunk_t* c) {
    return (void*)((char*)c + k_chunk_overhead);
}

static chunk_t* user_to_chunk(void* p) {
    return (chunk_t*)((char*)p - k_chunk_overhead);
}
```

先看 `malloc/free` 逻辑

```c
void* malloc_me(size_t n) {
    n = align_up(n == 0 ? 1 : n);

    if (n >= k_mmap_threshold) { // 页粒度内存申请，直接用 malloc
        return malloc_mmap(n);
    }

    chunk_t* c = find_fit(n); // 从 free list 中挑选
    if (c) {
        remove_free(c);
        split_if_needed(c, n);
		c->flags |= FLAG_IN_USE;
        return chunk_to_user(c);
    }

    if (!g_top || g_top->size < n) { // 查看 top chunk 的容量，按需增长
        if (!grow_top(n)) {
            return NULL;
        }
    }

    c = carve_from_top(n); // 从 top chunk 抠下来需要的 chunk 
    if (!c) {
        return NULL;
    }

    c->flags |= FLAG_IN_USE;
    return chunk_to_user(c);
}

void free_me(void* p) {
    if (!p) {
        return;
    }

    chunk_t* c = user_to_chunk(p);
    if ((c->flags & FLAG_IN_USE) == 0) {
        return;
    }

    c->flags &= ~FLAG_IN_USE;

    if (c->flags & FLAG_IS_MMAP) {
        munmap((void*)c, k_chunk_overhead + c->size);
        return;
    }

    c = coalesce_basic(c);

    if (absorb_into_top(c)) { // 如果在 top chunk 正下方，合并回 topchunk
        return;
    }

    insert_free(c); // 否则记录入 free list 
}
```

如果走 mmap 分配：

```c
static void* malloc_mmap(size_t want) {
    size_t total = k_chunk_overhead + want;
    void* mem = mmap(NULL, total, PROT_READ | PROT_WRITE,
                     MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    chunk_t* c;

    if (mem == MAP_FAILED) {
        return NULL;
    }

    c = (chunk_t*)mem;
    c->size = want;
    c->flags = FLAG_IN_USE | FLAG_IS_MMAP;
    c->prev_phys = NULL;
    c->next_phys = NULL;
    c->prev_free = NULL;
    c->next_free = NULL;

    return chunk_to_user(c);
}
```

小内存直接在 heap 按需分配。具体而言，每次申请内存，让 `heap` 向高内存增长，开辟出内存。可执行文件的内存布局可以参考 [Liunx 内存分布](../mem/linux-内存空间分布.md)

```
[heap]  
...     <---- heap 增长方向，堆顶是 brk

--------- 内存空洞 -----
[mmap]  <---- 按需分配，OS 决定分布


--------- 内存空洞 ------
...
...      <--- stack 增长方向，堆顶是 sp
[stack]
```

```c
static chunk_t* g_heap_head = NULL; // physic list head
static chunk_t* g_heap_tail = NULL; // physic list tail 
static chunk_t* g_free_head = NULL; // free list head
static chunk_t* g_top = NULL; // = g_heap_tail 

/*
	heap_head --> chunks ... -> heap_tail (top, brk) 
*/
```

在单线程 glibc ptmalloc 实现中，内存块链表尾，就是程序堆顶。每次申请小块堆内存时，增长堆顶。在多线程程序中，堆顶 `brk` 是共享状态资源，需要控制线程间数据竞争。

但是，如果用锁来管理会严重拖慢 `malloc` 速度。因此，一些第三方 `malloc` 实现已经全部使用 `mmap()` 来申请内存。

```c
// insert into the head of  free list 
static void insert_free(chunk_t* c);

// remove from free list (doubly list)
static void remove_free(chunk_t* c);

// find node with fit size in free list 
static chunk_t* find_fit(size_t want) {
    chunk_t* cur = g_free_head;
    while (cur) {
        if (cur->size >= want) {
            return cur;
        }
        cur = cur->next_free;
    }
    return NULL;
}

// 切除 chunk 多余部分，变为一个新 chunk
static void split_if_needed(chunk_t* c, size_t want);


// 增长 top chunk 体积。初次调用，初始化 top chunk 
// sbrk(n) 是原地推进 .data 段尾
static bool grow_top(size_t want) {
    size_t grow_payload = align_up(want > k_heap_grow ? want : k_heap_grow);
    size_t grow_total = k_chunk_overhead + grow_payload;
    void* mem;

    if (g_top) {
        mem = sbrk((intptr_t)grow_total);
        if (mem == (void*)-1) {
            return false;
        }
        g_top->size += grow_total;
        return true;
    }

    mem = sbrk((intptr_t)grow_total);
    if (mem == (void*)-1) {
        return false;
    }

    g_top = (chunk_t*)mem;
	*g_top = (chunk_t){
	    .size = grow_total - k_chunk_overhead,
	    .prev_phys = g_heap_tail,
	};

    if (g_heap_tail) g_heap_tail->next_phys = g_top;
    else g_heap_head = g_top;

    g_heap_tail = g_top;
    return true;
}

static chunk_t* carve_from_top(size_t want) {
    chunk_t* alloc;
    chunk_t* new_top;

    if (!g_top || g_top->size < want) {
        return NULL;
    }

    if (g_top->size < want + k_chunk_overhead + k_min_split) {
        alloc = g_top;
        g_top = NULL;
        return alloc;
    }

	// top chunk 过大，切割一部分
    alloc = g_top;
    new_top = (chunk_t*)(chunk_to_user(alloc) + want);
    *new_top = (chunk_t) {
	    .size = alloc->size - want - k_chunk_overhead,
	    .prev_phys = alloc,
    }

    alloc->size = want;
    alloc->next_phys = new_top;

    g_heap_tail = new_top;
    g_top = new_top;
    return alloc;
}

// 尝试和前后 chunks 合并
static Chunk* coalesce_basic(Chunk* c) {
    if (c->prev_phys 
		    && !(c->prev_phys->flags & FLAG_IN_USE)
		    && c->prev_phys != g_top
	) {
        Chunk* left = c->prev_phys;
        remove_free(left);

        left->size += k_chunk_overhead + c->size;
        left->next_phys = c->next_phys;

        if (c->next_phys) c->next_phys->prev_phys = left;
        else g_heap_tail = left;

        c = left;
    }

    if (c->next_phys 
		    && !(c->next_phys->flags & FLAG_IN_USE)
		    && c->next_phys != g_top
	) {
        Chunk* right = c->next_phys;
        remove_free(right);

        c->size += k_chunk_overhead + right->size;
        c->next_phys = right->next_phys;

        if (right->next_phys) right->next_phys->prev_phys = c;
        else g_heap_tail = c;
    }

    return c;
}

static bool absorb_into_top(Chunk* c) {
    if (g_top && c->next_phys == g_top) {
        c->size += k_chunk_overhead + g_top->size;
        c->next_phys = NULL;
        g_heap_tail = c;
        g_top = c;
        return true;
    }

    if (c->next_phys == NULL) {
        g_heap_tail = c;
        g_top = c;
        return true;
    }

    return false;
}

```

## jemalloc

## tcmalloc

## kernel kmalloc 

slub allocator