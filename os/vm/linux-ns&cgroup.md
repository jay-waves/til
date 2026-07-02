namespaces 用于系统资源隔离, cgroup 用于资源用量限制和追踪, 两者通常结合使用. 

## Namespace

```c 
struct task_struct {
	...
	struct nsproxy *nsproxy;
};

struct nsproxy {
	atomic_t count;
	
	struct uts_namespace *uts_ns; // 主机名隔离
	struct ipc_namespace *ipc_ns; 
	struct mnt_namespace *mnt_ns; // fs 挂载点隔离
	struct pid_namespace *pid_ns_for_children;
	struct net           *net_ns; 
	struct cgroup_namespace *cgroup_ns;
	struct time_namespace *time_ns;
};
```

```
task1
  └── nsproxy A
        ├─ uts_ns = U0
        ├─ pid_ns = P0
        ├─ net_ns = N0
```

NS 用户态命令是 `unshare`，用于隔离一个进程的资源：

```bash
unshare --pid --mount --uts --ipc --net --fork chroot /xxxx /bin/sh 
```
* `pid` 隔离进程 ID 空间
* `mount` 隔离硬盘加载表
* `uts` 隔离 hostname
* `ipc` 隔离 [IPC](../io/ipc/ipc.md) 
* `net` 隔离协议栈
* `chroot` 指定一个新的 rootfs ，比如自行下载一个 alphine linux

## CGroup 

cgroup (control groups) 是内核提供的资源控制工具 (限制和统计). 通过 `/sys/fs/cgroup` 下的接口进行控制.

controller:
- cpu/cpuacct, 限制 CPU 使用上限
- memory, 内存配额
- blkio, 块设备 IO 管理, 如限制 swap 和带宽
- pids, 限制进程数
- freezer, 冻结进程
- cpuset, 绑定 cpu 节点