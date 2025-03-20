Linux 内核中使用 `task_struct` 结构体来描述进程, 包含如下资源:
- 内存
- 文件系统和文件
- tty 资源
- 信号处理

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

linux 中线程和进程本质都是 task, 区别在于共享的资源不同.

## files_struct

```c
struct files_struct {
	struct fdtable *fdt;
	int count;
	fs_set close_on_exec;
	fs_set open_fds;
	struct file *fd[NR_OPEN];
}

struct fdtable {
	struct file **fd;
}

struct fs_struct {
	int count;
	unsigned short umask;
	struct inode *root, *pwd;
}
```

```c
/* file, dir, socket, block device, pipeline... */
struct file {
	struct file_operations *f_op;
	struct inode *f_inode;
	struct file *f_next, *f_prev;
	struct address_space *f_mapping;
	unsigned long f_version;

	int f_owner; /*pid or -pgrp where SIGIO should be sent */
	unsigned long f_read, f_ramax, f_raend, f_ralen, frawin;
	unsigned short f_count;
	unsigned short f_count;
	unsigned short f_flags;
	mode_t f_mode;
}

struct socket {

}
```

## address_space 

```c
struct address_space {
	struct inode *host;
	struct radix_tree_root page_tree;
	unsigned long nrpages;
	struct addres_space_operations *a_ops;
}

struct inode {
}
```

## mm_struct

描述一个进程的整个虚拟空间.

```c
// in include/linux/mm_types.h
struct mm_struct {
	int            count;
	pgd_t         *pgd;
	unsigned long  context;
	unsigned long  task_size; /* size of task vm space */

	unsigned long start_code, end_code; /* code */
	unsigned long start_data, end_data; /* data */
	unsigned long start_brk, brk;       /* heap */
	unsigned long start_stack;          /* stack */
	/* note stack top pointer is in sp register */

	/* pages count */
	unsigned long total_vm;    /* total pages mapped */
	unsigned long locked_vm;   /* pages cannot swapped out */
	unsigned long pinned_vm;   /* pages cannot swapped out or moved */
	unsigned long data_vm;       /* VM_WRITE & ~VM_SHARE & ~VM_STACK */
	unsigned long exec_vm;       /* VM_EXEC & ~VM_WRITE & ~VM_STACK */
	unsigned long stack_vm;      /* VM_STACK */

	unsigned long mmap_base;            /* base of mmap area */
	struct vm_area_struct *mmap; /* mmap, list head of VMAs */
	struct rb_root *mm_rb;       /* root for red-black tree of VMAs */
	struct list_head mmlist;    

}
```

> `unsigned long` 在 64 位机器上为 64 位, 在 32 位机器上表现为 32 位.   
> 适合用于标识地址.

## vm_area_struct 

描述一个连续的内存区域 (VMA, Virtual Memory Area), 一个进程的虚拟内存空间由多个 VMA 构成.

```c
struct vm_area_struct {

	struct mm_struct *vm_mm;	/* The address space we belong to. */
	
	unsigned long vm_start;   /* Our start address within vm_mm. */
	unsigned long vm_end;     /* The first byte after our end address within vm_mm. */
	
	/* linked list of VM areas per task, sorted by address, for quick iteration */
	struct vm_area_struct *vm_next, *vm_prev;

	/* Red-Black Tree of VMAs per task, sorted by address, 
	 * for quick search and insert 
	 * early version of kernel uses AVL tree.
	*/
	struct rb_node vm_rb; 
	
	/* for areas with inode, the circular list inode -> i_mmap */
	/* for shm areas, the circular list of attaches */
	/* otherwise unused */
	struct vm_area_struct *vm_next_share;
	struct vm_area_struct *vm_prev_share;

	/*
	 * Access permissions of this VMA.
	 */
	pgprot_t vm_page_prot;
	unsigned long vm_flags; 
	unsigned long vm_offset;
	struct inode *vm_inode;

	unsigned long vm_pte; /* shared memory */
	
	struct anon_vma *anon_vma;  /* Serialized by page_table_lock */
	struct list_head anon_vma_chain; 
	
	struct file * vm_file;      /* File we map to (can be NULL). */
	unsigned long vm_pgoff;     /* Offset (within vm_file) in PAGE_SIZE
										 units */ 
	void * vm_private_data;     /* was vm_pte (shared mem) */
	
	/* Function pointers to deal with this struct. */
	const struct vm_operations_struct {
		void (*open)(struct vm_area_struct *area);
		void (*close)(struct vm_area_struct *area);
		vm_fault (*fault)(struct vm_fault *vmf);
		...
	}*vm_ops;
}
```


### vm_flags

| vm_flags | control |
| --- | --- |
| `VM_READ` | 可读 |
| `VM_WRITE` | 可写 |
| `VM_EXEC` | 可执行 |
| `VM_SHAR` | 可多进程间共享 |
| `VM_IO` | 可映射至设备 IO 空间 |
| `VM_RESERVED` | 内存区域不可被换出 |
| `VM_SEQ_READ` | 内存区域可能被顺序访问 |
|`VM_RAND_READ` | 内存区域可能被随机访问 |

代码段的标志为 `VM_READ & VM_EXEC`. 

栈段标志可能为 `VM_READ & VM_WRITE`. 此时系统会赋予页 `p` 权限, (形如 `rw-p`), 指 COW (Copy on Write), 此时父子进程可共享相同的内存页, 标记为只读 (RO), 当一个进程试图修改此页面时, 会触发页错误, 为其创立独立的副本.