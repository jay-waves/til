https://docs.python.org/zh-cn/3.12/library/gc.html

https://zikcheng.com/2022-05-22-python-memory-allocator

https://docs.python.org/zh-cn/3.7/c-api/memory.html

> https://www.cnblogs.com/traditional/p/13698244.html
> https://www.cnblogs.com/traditional/p/13695846.html

Python 是一门自动内存垃圾回收语言 (Garbage Collection), 以 CPython 实现为例, 其内存管理层次为:

| 层次                 | 功能               | 实现方式 |
| -------------------- | ------------------ | -------- |
| 特定对象内存分配器   | 不同对象的缓存池   | Python   |
| Python 对象分配器    | 对象内存, 内部缓冲 | Python   |
| Python 内存分配器    | PyMem_ API         | Python   |
| 通用内存分配器       | C API (malloc...)  | C        |
| 操作系统虚拟内存管理 | 内核页映射         | 操作系统 |
| 底层存储设备         | 主存, 高速缓存     | 操作系统         |

## Python 内存分配器

`PyMem_` 系列调用接口封装了底层内存管理接口, 旨在兼容不同平台的 ANSI C 标准接口的差异性, 定义统一的行为.

```c
//Include/pymem.h
PyAPI_FUNC(void *) PyMem_Malloc(size_t size);
PyAPI_FUNC(void *) PyMem_Realloc(void *ptr, size_t new_size);
PyAPI_FUNC(void) PyMem_Free(void *ptr);


//Objects/obmalloc.c
void *
PyMem_Malloc(size_t size)
{
    /* see PyMem_RawMalloc() */
    if (size > (size_t)PY_SSIZE_T_MAX)
        return NULL;
    return _PyMem.malloc(_PyMem.ctx, size);
}

void *
PyMem_Realloc(void *ptr, size_t new_size)
{
    /* see PyMem_RawMalloc() */
    if (new_size > (size_t)PY_SSIZE_T_MAX)
        return NULL;
    return _PyMem.realloc(_PyMem.ctx, ptr, new_size);
}

void
PyMem_Free(void *ptr)
{
    _PyMem.free(_PyMem.ctx, ptr);
}
```

Python 还提供了面向对象风格的内存分配 `PyMem_New()`, 即无须手动提供所需空间大小, 只需提供类型和数量.

```c
//Include/pymem.h

#define PyMem_New(type, n) \
  ( ((size_t)(n) > PY_SSIZE_T_MAX / sizeof(type)) ? NULL :      \
        ( (type *) PyMem_Malloc((n) * sizeof(type)) ) )

#define PyMem_Resize(p, type, n) \
  ( (p) = ((size_t)(n) > PY_SSIZE_T_MAX / sizeof(type)) ? NULL :        \
        (type *) PyMem_Realloc((p), (n) * sizeof(type)) )
        
#define PyMem_DEL               PyMem_Free
```

## Python 对象分配器

该层为 Python 主要的内存管理层, 用于创建 Python 对象 (PyObject). 该层函数以 `PyObject_` 开头. Python GC 机制主要位于这一层.

Python 运行期间需要频繁申请和释放小块内存, 为了避免大量执行底层系统调用 `malloc/free` 拖慢性能, Python 引入了**内存池机制** (PyMalloc). 当内存小于等于 512B 时, 内存分配会被 Python 内存池接管; 大于 512B 时, 才使用系统的 `malloc` 调用.

```c
//Objects/obmalloc.c
#define ALIGNMENT               8  /* block 对齐字节 */
#define SMALL_REQUEST_THRESHOLD 512
#define NB_SMALL_SIZE_CLASSES   (SMALL_REQUEST_THRESHOLD / ALIGNMENT) /* block 也称 size class */
```

内存池有三层结构, 从下至上依次是: `block, pool, arena`.

### block

Python 不可能为每个尺寸的内存都准备独立内存池, 所以以 8 字节为梯度, 分为了 64 种内存池单元 `block`. 

### pool

block 的集合称为 pool. pool 管理着一大块内存, 大小通常为 4KB.

```c
//Objects/obmalloc.c
#define SYSTEM_PAGE_SIZE        (4 * 1024)
#define SYSTEM_PAGE_SIZE_MASK   (SYSTEM_PAGE_SIZE - 1)
#define POOL_SIZE               SYSTEM_PAGE_SIZE        /* must be 2^N */
#define POOL_SIZE_MASK          SYSTEM_PAGE_SIZE_MASK
```

CPython 定义的 Pool 结构如下:

```c
//Objects/obmalloc.c
/* Pool for small blocks. */
struct pool_header {
    union { block *_padding;
            uint count; } ref;          /* 当前pool里面已分配出去的block数量 */
    block *freeblock;                   /* 指向空闲block链表的第一块 */
    
    /* 底层会有多个pool, 多个pool之间也会形成一个链表 */
    struct pool_header *nextpool;       /* 所以nextpool指向下一个pool */
    struct pool_header *prevpool;       /* prevpool指向上一个pool */
    uint arenaindex;                    /* 在area里面的索引(area后面会说) */
    uint szidx;                         /* 尺寸类别编号, 如果是2, 那么管理的block的大小就是24 */
    uint nextoffset;                    /* 下一个可用block的内存偏移量 */
    uint maxnextoffset;                 /* 最后一个block距离开始位置的偏移量 */
};

typedef struct pool_header *poolp;
```

### arena



## Python 特定对象内存分配器

用于对象的缓存机制, 如小整数对象池, 浮点数缓存池等.