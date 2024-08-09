
## 操作系统 特征

**操作系统是**:   
1. 计算机**最基本**系统软件
2. 抽象, 控制和管理系统的**硬件和软件资源**, 组织调度计算机工作和资源分配.
3. 为用户和软件提供**接口与环境**的程序集合, 如 CLI, GUI, API.

其中**并发和共享**是基本特征.

### 并发 Concurrence

并行性: 多个事件同一时刻发生.

并发性: 多个事件宏观上同时发生, 微观上交替发生.

CPU单核通过**分时**实现并发, 通过多核实现并行.

### 共享 Sharing

互斥共享: 一段时间只允许一个进行访问某资源 (称为**临界资源**). 如麦克风, [1 变量](../Shell/sh%20script/1%20变量.md)等.

同时访问方式: 一段时间允许多个进程"同时"访问 (虽然微观可能仍是分时访问的). 比如磁盘.

### 虚拟 Virtual

如虚拟内存, 虚拟处理机和虚拟外部设备. 让用户感觉自己独自占有使用某资源.

### 异步 Asynchronism

资源有限与处理分时, 导致程序以不可预知的速度向前推进.


### 操作系统架构

| 用户       |            |            |
| ---------- | ---------- | ---------- |
| 非内核功能 |            |            |
| 进程管理   | 存储器管理 | IO设备管理 |
| 时钟管理   | 中断处理   | 原语操作   |
| 硬件           |            |            |

[Linux 组成](Linux.md):
1. 内核
2. Shell
3. 文件系统
4. 应用程序

| Linux 系统架构 | 安卓系统架构 |
| -------------- | ------------ |
|    ![\|200](../../attach/Pasted%20image%2020240429171126.png)            |  ![安卓系统五层架构\|200](../../attach/Pasted%20image%2020230620164006.png)            |



***


## System Startup 

`init` initialization code from kernel 

## Memory Management

page fault hadling in `mm/memory.c`, memory mapping and page cache in `mm/filema.c`, buffer cache in `mm/buffer.c`, swap cache inn `mm/swap_stat.c, mm/swapfile.c`

## Process Management

scheduler in `kernel/sched.c`, fork in `kernel/fork.c` 

### IPC

`ipc` inter-prcess communication 

### Interrupt Handling

almost platform specific, like `arch/i386/kernel/irq.c` and `include/asm-i386/irq.h`

## PCI

PCI pseudo driver in `drivers/pci/pci.c` and `include/linux/pci.h`

## Device Drivers

- block: like ide.c, device_setup() in `drivers/block/genhd.c`
- char: character-bases devices, like ttys
- cdrom: CDROM
- pci
- scsi
- net
- sound

## File Systems

`fs` file system, like `ext4`, `vfat`

ext2 in `fs/ext2/` and `include/linux/ext2_fs.h, ext2_fs_i.h, ext2_fs_sb.`

VFS in `include/linux/fs.h` and `fs/*`

buffer cache in `fs/buffer.c`

## Network

`net`, `inlucde/net` network

BSD socket in `net/socket.c`

IPv4 in `net/ipv4/af_inet.c`

tcp/ip in `net/ipv4`

drivers in `dirvers/net`

***


`arch` architecture specific code, like `i386`, `x86`, `arm`

`include`, header files



`mm` memory management, architecture specific memory management code lives down `arch/*/mm`



`kernel` main kernel code, like **system call**, architecture specific in `arch/*/kernel`



`lib` library code, architecture specific in `arch/*/lib`

### 系统调用接口

从用户空间到内核的函数调用:
- 实现: `./linux/kernel`
- 依赖于体系结构的部分: `./linux/arch`

### 内存管理

Linux 使用虚拟内存技术, 将内存划分为 4KB 大小的内存页, 并提供寻址技术和交换技术.

实现: `./linux/mm`


### 进程管理

进程是应用程序的一个运行实体, Linux 通过分配时间片"并发"执行进程, 完成进程调度.

Linux 通过优先队列来调度进程.

SCI 提供了 `fork, exec, kill, wait, exit` 等 API, 并提供了信号机制.


### 设备驱动

实际操控操作系统与硬件设备交互, 运行在高特权级的环境中, 驱动错误很可能导致OS崩溃.

这都是什么??????