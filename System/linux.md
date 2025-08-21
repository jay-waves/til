
[Linux 组成](linux.md):
1. 内核
2. Shell
3. 文件系统
4. 应用程序

| Linux 系统架构 | 安卓系统架构 |
| -------------- | ------------ |
|    ![\|500](../../attach/Pasted%20image%2020240429171126.avif)            |  ![安卓系统五层架构\|500](../../attach/Pasted%20image%2020230620164006.avif)            |

Linux 内核组成: (五个子系统)
- SCHED 进程调度. 
- MM 内存管理. 和进程调度系统的耦合度较高.
- VFS 虚拟文件系统
- NET 网络接口
- IPC 进程间通信.

## Kernel 

Kernel 工作在三个情景下:
- system calls from userspace. 
- interrupt handlers triggered by hardware. 
- long-lived kernel threads (wakeup)
- exceptions from traps and faults

内核线程安排如下:
- 第一个内核线程是 `init` (`systemd`), `pid = 1`, 启动 Userland
- 第二个内核线程是: `kthreadd`, `pid = 2`, 创建所有其他内核线程. 
- 其他内核线程:
	- 每个 CPU 核有独立的: IRQs, watchdog, migration helper, worker queue 
	- 视需求有: I/O, memory management, filesystem, device drivers. 

> "Kernel is system's core, always resident, always privileged, and always in control".

## Execution Model 

- Process Context: 用户态, 通过系统调用陷入内核. 拥有完整虚拟地址空间.
- Kernel Thread: 内核态, 本质是调度实体 `task_struct`, 但没有独立的用户态虚拟地址空间. 不能直接访问用户空间的数据.
- SoftIRQ / Tasklet: 延迟执行的中断处理程序, 禁止睡眠或阻塞. Tasklet 是对 SoftIRQ 的封装, 但调度方式不同.
- IRQ Handler: 硬件中断处理函数, 不参与调度, 直接抢占 CPU. 不允许睡眠或阻塞, 也禁止使用可能触发调度的函数 (mutex, kmalloc) 等. 复杂逻辑应推迟到 SoftIRQ.

内核延迟执行的流水线:

IRQ Handler -> SoftIRQ -> Workqueue -> Kernel Tread

| 执行器  | 上下文         | 是否可睡眠 | 调度         | 内存                   |
| ------- | -------------- | ---------- | ------------ | ---------------------- |
| Process | 用户/内核态    | 是         | 内核调度     | 独立用户态虚拟地址空间 |
| KThread | 内核态         | 是         | 内核调度     | 仅内核地址空间         |
| IRQ     | 中断上下文     | 禁止       | 抢占执行     | GFP_ATOMIC             |
| SoftIRQ | 软中断上下文   | 禁止       | 内核触发     | GFP_ATOMIC             |
| Tasklet | SoftIRQ 上下文 | 禁止       | 串行化软中断 | GFP_ATOMIC                       |

在 IRQ, SoftIRQ, Tasklet 执行, **禁止调用可能触发调度的函数**, 防止当前 CPU 放弃执行切换到其他 task. 比如阻塞等待类 `wait_*(), down(), mutex_lock()`, 可能导致睡眠的内存分配 `kmalloc(GFP_KERNEL), vmalloc()`, 可能触发缺页中断的 `copy_to_usr(), vfs_write()` 等. **如果导致内核调度到其他上下文, IRQ 等上下文是无法再恢复的, 导致系统栈状态损坏, 进而崩溃.**

明确不会触发调度的函数有:
- 原子内存分配 `kmalloc(GFP_ATOMIC)`
- 自旋锁 `spin_xxx()`
- 原子操作 `atomic_inc()`, ...
- 延迟执行: `tasklet_shcedule(), raise_softirq()`

### System Calls Execution Flow 

1. User Space Process 
	- issue a syscall (`read(), write(), execve()`)
	- use architectur-specific instruction (e.g. `syscall` on x64)
	- switch from ring n to ring 0
2. Syscall Entry (Architecture-Specific)
	- enter via syscall handler (e.g. `entr_SYSCALL_64` on x64)
	- save user registers, set up kernel stack 
3. Syscall Dispatch Table 
4. Device / File Backend 
	- Filesystem (e.g. `ext4`)
	- Device driver (e.g. `block, pipe, char`)
	- Socket/NIC (e.g. `net`)
5. Return To Userspace (Exit Path)
	- copy result to userspace buffer, restore registers
	- switch from ring 0 to ring n

### Internal Kernel Thread & System Maintenance 

1. Kernel Thread Creation (Boot, or Runtime)
	- `kthread_create()`
	- associated with a `stack_struct`
2. Kernel Thread Main Event Loop 

### Hardware Interrupt Handling 

1. Hardware Device (NIC, Disk)
	- I/O done, raised an IRQ (interrupt line)
	- Interrupt Controller (APIC/IOAPIC) signals the CPU 
2. CPU Interrupt Handler (Low-Level) 
	- CPU is interrupted (preemption of running_task)
	- Enters arch-level IRQ entry (some vector table)
	- Disables local IRQs, saves context, acknowledges interrupt 
	- Calls the registered high-level IRQ 
3. TOP-HALF IRQ Handler 
	- Minimal, non-blocking, fast-path logic 
	- Reads device status / clears flags 
	- Schedules deferred work via softirq, tasklet, or workqueue 
4. Deferred Work: Bottom-Half
	- Allow sleeping / locking / shceduling if needed 
	- performs bulk work outside hard IRQ context
5. Kernel Subssytems Handle Final Processing 
6. Return to Previous Context 
	- restore task that war interrupted 
	- resume normal kernel or user-space execution 

E.G. Network Stack (NIC):  
packet received --> IRQ (Top-Half) --> SoftIRQ (Bottom-Half) --> NAPI Polling --> build `sk_buff`, deliver to socket buffer --> idle 

E.G. Disk:  
I/O done --> IRQ (Top-Half) --> Workqueue (Bottom-Half) --> Workqueue --> complete I/O req, update page cache --> idle 

## Kernel Objects 

这里按子系统列出一些常见内核对象. 具体细节和成员含义, 请参见具体的场景.

进程信息: 用 `unsigned long` 表示地址, 64b 机器上为 64b, 32b 机器上为 32b.
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

/*
	include/linux/sched.h
*/
struct task_struct {
	pid_t pid;
	long state;
	void *stack; 
	struct mm_struct *mm;       // 进程地址空间
	struct files_struct *files; // 打开的文件表
	struct task_struct *parent;
};
```

文件系统:
```c
/*
	万物皆文件: file, dir, socket, block device, pipeline...
	所有文件都是 inode

	include/linux/fs.h
*/
struct inode {
	unsigned long ino;  // inode 编号
	struct super_block *sb;  // 实际代表了文件系统类型
};

/*
	目录项, 缓存路径名到 inode 的映射.

	include/linux/dcache.h
*/
struct dentry {
	const char* name;
	struct inode *inode; // 文件实体
	struct dentry *parent;
};

/*
	include/linux/fs.h
*/
struct file {
	struct dentry          *dentry; // 代表对文件路径的解析
	struct file_operations *f_op; 
	struct inode           *f_inode;
	struct address_space   *f_mapping;

	int            f_owner;
	unsigned short f_count;
	unsigned short f_flags;
	mode_t         f_mode;
};

/*
	include/linux/fdtable.h
*/
struct files_struct {
	struct file    *fd[NR_OPEN]; // 进程打开的文件表
	int             count;
	struct fdtable *fdt;  /* struct file **fd */
	fs_set          open_fds;
};

struct super_block {
	const char *fs_type;
};

struct fs_struct {
	int count;
	unsigned short umask;
	struct inode *root, *pwd;
};
```

设备驱动

```c
struct device {
	const char *name;
	struct device_driver *driver;
};

struct devcie_driver {
	const char *name;
	struct file_operations *fops; // 文件系统接口的实际实现
}

struct file_operations {
	ssize_t (*read)(struct file *, char __user *, sizre_t);
	...
}
```

网络协议栈

```c
/*
	include/linux/sk_buff
*/
struct sk_buff {
	void *data;
	unsigned int len;
	struct sk_buff *next; 
};

/*
	include/linux/sock.h
	表示具体的协议实现, 并且保存协议的状态机
*/
struct sock {
	int state;
	struct sk_buff *rx_queue, *tx_queue;

	struct proto *sk_prot; // proto 具体实现
	void *sk_protinfo;     // proto 私有数据
}

/*
	VFS 层的抽象, 和文件系统强行连接在一起.
*/
struct socket {
	
	socket_state state;
	struct file *file;
	struct sock *sk;             // 具体协议实现
	const struct proto_ops {
		/* 
			协议操作表, 就是 socket() 调用的虚函数表
			具体的协议实现, 会各自注册一套 proto_ops 
		 */
		int (*bind)(struct socket *, struct sockaddr *, int);
		int (*connnect)(....);
		int (*listen)(...);
		...
	} *ops;
	
};
```

内存页

```c
struct page {
	unsigned long flags;
	void *addr;
};

struct address_space {
	struct inode                    *host;
	struct radix_tree_root           page_tree;
	unsigned long                    nrpages;
	struct address_space_operations *a_ops;
};
```