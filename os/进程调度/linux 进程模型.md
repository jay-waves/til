Linux 内核的进程上下文包括:
- `task_struct`: 大部分进程资源
	- `mm_struct` 虚拟内存
	- `fs_struct` 目录
	- `files_struct` 文件描述符表
	- `signal_xxx` 信号处理
	- `cred` 鉴权
	- `thread_struct`: 保存和硬件相关的信息, 如 CPU 状态 / 内核栈指针等.

Linux 线程采用 轻量级进程模型 实现, 调用 `pthread_create()` 创建线程时, 实际创建了新的 `task_struct`, 同时共享其父 `task_struct` 的部分资源指针.

## task_struct

```c
struct task_struct {
	pid_t                pid;   /* process id */
	pid_t                tgid;
	long                 state; /* process state */
	void                *stack; /* stack pointer */
	int                  prio, static_prio;
	struct files_struct *files; /* files opened by thread. */
	struct mm_struct    *mm;
```

linux 中线程和进程本质都是 task, 区别在于共享的资源不同. 在陷入内核时, 通过 `current` 宏来获取当前进程的 `task_struct`, 复杂度为 `O(1)`. 

在大部分架构 (x86, arm, risc-v) 中, 每个进程都有一个私有*内核栈*, 内核栈中存放 `thread_info`, 其中存放 `task_struct`. 该数据结构压在内核栈底, 通过栈基地址即可获取.

```c
syscall() 
	--> entry_SYSCALL_64()  // 汇编入口, 保存上下文
		--> do_syscall_64() 
			--> current = get_current() // 获取当前 task_struct 
				--> xxx() // 执行具体系统调用
```

## fork()

Linux 创建新进程时, 并不完整复制原进程, 而是和原进程共享一个*写时复制 (Copy on Write, COW)* 的内存空间. 

新进程有新的 `task_struct`, 大部分原信息是复制过来的, 但内存是浅拷贝的: 虚拟内存 VMA 复制给子进程, 但不会立刻复制物理页. 所有共享的物理页 PTE 立刻被标记为只读, 清空"可写"标志, 然后更新 `struct page` 中的 `refcount`.

一旦某进程对某个只读 PTE 进行修改, 就会触发*缺页异常 (Page Fault)*, 会分配一个新的物理页, 然后把旧页复制到新页. 