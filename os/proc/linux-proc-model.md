
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

### 和 VFS 的关系

![task_struct](../../attach/ascii/task_struct.md)

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

* `pid` 进程 PID，也叫 TID。多指线程，对应某个 `task_struct`。
* `tgid` 线程组 TGID。多指进程。
* `ppid` 父进程 PPID。
* `pgid` 进程组 PGID。多用于 Shell Job Control
* `sid` 会话 SID。多指某个 终端/用户登陆 的单次会话。

当进程创建多个线程时，多个线程的 `pid` 各不相同，而 `tgid` 相同，都继承自初始线程的 `pid` 。
在用户态调用 `getpid()` 实际返回了 `tgid`，调用 `gettid()` 返回 `pid` 。`ppid` 默认指向 
`fork()` 父进程，当父进程退出后，子进程不会退出，而是将 `ppid` 指向父进程的父进程，其他复杂行为略。
用户态调用 `getppid()` 返回 `tgid` 而不是实际的父线程 `pid`，但内核里复杂一些。

多个进程可以合并为进程组，有 `pgid`；多个进程组归入一个 Session 中，有 `sid (session id)` 。

引入 PID Namespace 后，PID/TGID/SID/PGID 都只在命名空间内保持唯一，计算更复杂了。


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

> 这种混乱是因为历史进程控制机制叠在了一起：fork/exec/wait 一套、job control/Ctrl-C/pipeline 一套、
> task/thread 一套、namespace/container 一套。
