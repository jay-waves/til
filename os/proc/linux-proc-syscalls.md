## fork()

Linux 创建新进程时, 并不完整复制原进程, 而是和原进程共享一个*写时复制 (Copy on Write, COW)* 的内存空间. 

新进程有新的 `task_struct`, 大部分原信息是复制过来的, 但内存是浅拷贝的: 虚拟内存 VMA 复制给子进程, 但不会立刻复制物理页. 所有共享的物理页 PTE 立刻被标记为只读, 清空"可写"标志, 然后更新 `struct page` 中的 `refcount`.

一旦某进程对某个只读 PTE 进行修改, 就会触发*缺页异常 (Page Fault)*, 会分配一个新的物理页, 然后把旧页复制到新页.

#### CoW 

当内存页被两个进程共享时，会被标记只读。如果某进程试图写入，会触发[缺页异常](../mem/virtual-mem.md)。


#### do_fork()

`fork()`, `vfork()`, `clone()` 最终都会调用 `do_fork()` ，由它调用 `copy_process()`
* 复制 `task_struct` ，结构详见 [linux-进程](linux-proc-model.md)
* 建立新的内核栈 `thread_union` 。 `task_struct->task` 引用了该结构内的 `stack`。
* 提供了很多 `CLONE_XXX` falgs, 以及对应函数 `copy_xxx()`. 主要控制父子进程间共享哪些资源
* 填充 `task_struct->task_thread` 

```c
// sched.h
union thread_union {
	struct thread_info thread_info;
	unsinged long stack[THREAD_SIZE/sizeof(long)]; 
};

struct thread_info {
	struct task_struct *task; 
	struct exec_domain *exec_domain; 
	
	unsigned long status; 
	__u32         cpu; 
	int           preempt_count; // 0 --> 可抢占
	mm_segment_t  addr_limit;  
};
```

大部分体系结构中，会用两个内存页保存 `thread_union` 这个结构，因此内核栈的大小略小于 `8KiB` 

## execve()

`execve()` 调用和系统结构相关的 `sys_execve()` ，然后委托给结构无关的 `do_execve()` 
* 将可执行文件加载到内存，生成一个 fd 
* bprm_init 生成地址空间 `mm_struct` 然后传递其他参数
* 根据可执行文件的类型（如 ELF）具体处理 
* 释放原进程所有资源 


```c
int do_execve(char * filename, char **argv, char **envp, struct pt_regs *regs);
```

Linux 主要支持如下几种可执行文件：
* `flat_format` 用于没有 MMU 的裸机程序
* `script_format` 伪二进制格式，特点是开头的魔数 `#!`
* `elf_format` 

可执行文件需要被处理为：

```c
struct linux_binfmt {
    struct linux_binfmt *next;
    struct module *module;
    int (*load_binary) (struct linux_binprm *, struct pt_regs * regs);
    int (*load_shlib）(struct file *);
    int (*core_dump) (long signr, struct pt_regs * regs, struct file* file);
};
```

## exit()

进程必须用 `exit()` 终止，此时内核会使用资源。
