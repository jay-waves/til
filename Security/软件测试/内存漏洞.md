## 1. 越界访问

Out-ofBounds Access. 访问数组或内存块边界之外.

### 1.1. 堆栈外访问

访问了栈或堆的负偏移地址. (Stack/Heap Underflow) 

### 1.2. 缓冲区溢出

Buffer Overflow. 写数据超出预分配的缓冲区范围. 

- 栈溢出 (Stack Overflow): 递归函数过深, 或不安全写入函数 (如 `put()`)
- 堆溢出 (Heap Overflow): 堆段的溢出.
- 全局变量溢出 (Clobal Buffer Overflow): BSS 段的溢出.

栈溢出保护: `gcc -fstack-calsh-protection`

## 2. 无效地址

### 2.1. 空指针解引用

Null Pointer Dereference, 空指针 (悬挂指针, 野指针, Dangling Pointer) 解引用. 空指针指向了已经被释放的区域.

### 2.2 双重释放

Double Free. 尝试释放同一块内存两次.

### 2.3. 释放后使用 

- Use After Free (UAF)
- User After Return (UAR)
- Use After Scope (UAS)

### 2.4. 总线错误

SIGBUS, Bus Error

- 访问未对齐内存 (Misaligned Memory Access)
- 对只读区 (ROM) 执行写操作
- 执行了不可执行内存区段

## 3. 内存未初始化

Use of Uninitialized Memory. 未初始化内存区域可能有未定义数据.

- 未初始化变量读取
- 函数返回未初始化值
- 结构体或数组中部分未初始化成员的范围跟

## 4. 内存泄露

Memory Leak. 一般指未释放声明过的内存.

### 4.1 作用域错误

错误使用内存作用域, 导致内存泄露.  

```c
int* foo() {
    int a;          // 变量a的作用域开始
    a = 100;
    char *c = "xyz";   // 变量c的作用域开始
    return &a;
}                   // 变量a和c的作用域结束
```

这段代码有两个严重问题: 

1. a 指向的内存在函数结束时就会被释放掉, 返回的指针其实是悬空的. 
2. c 在常量区域新增了内存, 但是函数结束后不会再被使用, 这部分空间实际上浪费掉了. 

### 4.2 句柄泄露

句柄 (Handle) 指系统分配的资源 (文件句柄, 套接字, 线程)

### 4.3 堆内存泄露

动态分配的堆内存未释放 (heap leak)

### 4.4 全局或静态内存泄露