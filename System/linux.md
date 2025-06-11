
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

## Kernel 

Kernel 工作在三个情景下:
- system calls from user processes 
- interrupt handlers triggered by hardware 
- long-lived kernel threads (wakeup)

- 第一个内核线程是 `init` (`systemd`), `pid = 1`, 启动 Userland
- 第二个内核线程是: `kthreadd`, `pid = 2`, 创建所有其他内核线程. 
- 其他内核线程:
	- 每个 CPU 核有独立的: IRQs, watchdog, migration helper, worker queue 
	- 视需求有: I/O, memory management, filesystem, device drivers. 

> "Kernel is system's core, always resident, always privileged, and always in control".

### System Calls Execution Flow 

1. User Space Process 
	- issue a syscall (`read(), write(), execve()`)
	- use architectur-specific instruction (e.g. `syscall` on x64)
	- switch from ring n to ring 0
2. Syscall Entry (Architecture-Specific)
	- enter via syscall handler (e.g. `entr_SYSCALL_64` on x64)
	- save user registers, set up kernel stack 
	- check syscall number and dispatch to syscall table 
3. Syscall Dispatch Table 
	- Indexed by syscall number 
	- points to implementation function in kernel (e.g. `sys_read(), sys_execve(), sys_open()`)
4. Device / File Backend 
	- Filesystem (e.g. `ext4`)
	- Device driver (e.g. `block, pipe, char`)
	- Socket/NIC (e.g. `net`)
5. Return To Userspace (Exit Path)
	- copy result to userspace buffer 
	- restore registers
	- switch from ring 0 to ring n

### Internal Kernel Thread & System Maintenance 

1. Kernel Thread Creation (Boot, or Runtime)
	- `kthread_create()`
	- associated with a `stack_struct`
2. Kernel Thread Main Event Loop 

### Hardware Interrupt Handling 

1. Hardware Device (NIC, Disk)
	- Device completes an operation (I/O done, packet received )
	- Raised an interrupt line (IRQ)
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
- `irq` 中断服务
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

