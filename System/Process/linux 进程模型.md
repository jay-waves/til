Linux 内核中使用 `task_struct` 结构体来描述进程, 包含如下资源:
- 内存 `mm_struct`
- 文件系统和文件 `files_struct`
- tty 资源
- 信号处理
- 鉴权 `cred`

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


