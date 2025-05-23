
[Linux 组成](Linux.md):
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

## Linux 内核源码目录组织

- `arch` 硬件架构
	- `i386` interrupt: `arch/i386/kernel/irq.c`, `include/asm-i386/irq.h`
	- `x86`
	- `arm`
- `block` 块设备驱动程序 IO 调度
- `crypto` 密码算法, 压缩算法等
- `documentation`
- `drivers` 设备驱动程序, 如 `char, block, net, mtd`
	- PCI Pseudo Driver: `drivers/pci/pci.c`, `include/linux/pci.h`
- `fs` 各文件系统
	- `ext4`
	- `vfat`
	- vfs: `fs/*`, `include/linux/fs.h`
	- buffer cache: `fs/buffer.c`
- `include` 内核 API 级头文件. 系统相关头文件放在 `include/linux`
- `init` 内核初始化代码. 如 `init/main.c`
- `ipc` 进程间通信代码
- `kernel` 内核核心代码. 平台相关的则放在各个 `arch/*/kernel` 下
	- system call 
	- scheduler: `kernel/sched.c`
	- fork: `kernel/fork.c`
- `lib` 库文件代码. 平台相关在 `arch/*/lib`
- `mm` 内存管理. 平台相关放在 `arch/*/mm`
	- page fault handling: `mm/memory.c`
	- memory mapping and page cache: `mm/filema.c`
	- buffer cache: `mm/buffer.c`
	- swap cache: `mm/swap_stat.c`, `mm/swapfile.c`
- `net` 网络
	- BSD Socket: `net/socket.c`
	- IPv4: `net/ipv4/af_inet.c`
	- TCP/IP: `net/ipv4`
	- Net Drivers: `dirvers/net`
- `scripts` 用于配置内核的脚本
- `sound` 音视频驱动
- `usr`


