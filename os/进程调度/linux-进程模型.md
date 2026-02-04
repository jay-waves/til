
linux 中线程和进程本质都是 `task_struct`, 区别在于共享的资源不同. 在陷入内核时, 通过 `current` 宏来获取当前进程的 `task_struct`, 复杂度为 `O(1)`. 

## task_struct

```c
struct task_struct {

	long                 state; /* process state */
	void                *stack; /* stack pointer */
	int                  prio, static_prio;

	pid_t                pid;   /* process id */
	pid_t                tgid;
	struct task_struct *real_parent; // 用于调试
	struct task_struct *parent; 
	struct list_head children; // 子进程链表
	struct list_head sibling;  // 连接到父进程的子进程链表
// };
```

硬件相关信息：

```c
// struct task_struct {
	struct thread_struct thread;
// }
```

鉴权信息 `cred`：

```c
// struct task_struct {
	uid_t uid, euid, suid, fsuid;
	gid_t gid, egid, sgid, fsgid;
	struct group_info *group_info;
	
	struct user_struct *user;
// }
```

调度：

```c
// task_struct {
	struct list_head run_list;
	const struct sched_class *sched_class;
	struct sched_entity se;
	
	struct list_head tasks; 
//
```

内存：

```c
// task_struct {
	struct mm_struct *mm, *active_mm;
//
```

文件系统相关结构：

```c
// task_struct {
	struct fs_struct *fs; // 文件系统信息，如目录位置
	struct files_struct *files; /* 所有打开的文件 */
	int link_count, total_link_count; 
	
//
```

**资源控制**：比如打开文件的最大数目，就是通过 `rlim[RLIMIT_NOFILE]` 配置，默认为 `1024`；单用户的最大进程数，则用 `rlim[RLIMIT_NPROC]` 控制。详细信息可查看 `/proc/self/limits`。

命名空间机制详见：...

```c
// task_struct {
	struct rlimit[...] rlim;
	
	// 命名空间
	struct nsproxy *nsproxy;
// };

struct rlimit {
	unsigned long rlim_cur;
	unsigned long rlim_max;
};
```

信号处理：

```c
// task_struct {
	struct signal_struct* signal;
	struct sighand_struct *sighand;
	...
// }
```

## 内核栈

在大部分架构 (x86, arm, risc-v) 中, 每个进程都有一个私有*内核栈*, 内核栈中存放 `thread_info`, 其中存放 `task_struct`. 该数据结构压在内核栈底, 通过栈基地址即可获取.

```c
syscall() 
	--> entry_SYSCALL_64()  // 汇编入口, 保存上下文
		--> do_syscall_64() 
			--> current = get_current() // 获取当前 task_struct 
				--> xxx() // 执行具体系统调用
```

## 进程的关系

* `pid` 进程 PID
* `tgid` 线程组 TGID，进程无线程时，PID 和 TGID 相等。
* （略）进程可合并入进程组，进程组可合并入会话（session）

```c
// pid_namespace.h
struct pid_namespace {
...
	struct task_struct* child_reaper;
	...
	int level;
	struct pid_namespace *parent;
};

// pid.h
struct pid {
	atomic_t count;
	struct hlist_head tasks[PIDTYPE_MAX]; // 使用该 pid 的进程列表
	int level;
	struct upid numbers[1];
	struct upid nums[MAX_PID_NS_LEVEL];
};

struct upid {
	int nr; // 同一个 namespace 下的唯一 pid 编号
	struct pid_namespace *ns;
	struct hlist_node pid_chain;
};
```