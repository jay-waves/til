
## 操作系统的特征

**操作系统是**:   
1. 计算机**最基本**系统软件
2. 抽象, 控制和管理系统的**硬件和软件资源**, 组织调度计算机工作和资源分配.
3. 为用户和软件提供**接口与环境**的程序集合, 如 CLI, GUI, API.

其中**并发和共享**是基本特征.

### 并发 Concurrency

操作系统需要**同时执行多个任务**, 如果多个任务在微观上是同时发生的, 称为**并行**; 如果多个任务在宏观上是同时发生, 但微观是在有限的处理资源下交替分时发生, 则称为[**并发**](Process/进程与线程.md).

- 指令级并行: 单处理器通过[**分时**](Process/进程调度.md)实现并发, 核心是流水线技术.
- 数据级并行: 核心是多处理器并行技术.
- 线程级并行: [进程同步与互斥](Process/进程同步与互斥.md)

### 异步 Asynchronism

由于资源有限和处理分时性, 操作系统**无法预知任务的执行顺序, 因此以事件驱动的方式进行调度**, 通过[调度](Process/进程调度.md)和[中断](Process/中断.md)机制来确保各个任务不会互相阻塞, 阻塞任务让权等待. 

### 虚拟 Virtual

将物理资源**抽象**为虚拟资源, 透明化底层的复杂性. **隔离**某些非临界资源, 让用户和进程在逻辑上独占资源.  

- 虚拟化 CPU
- 虚拟化内存
- 虚拟化外部设备

### 共享 Sharing

一些临界资源无法被彻底虚拟化 (如 CPU, 文件, IO 设备, 已满负荷的内存), 需要多个用户和进程轮流使用 (互斥使用), 操作系统负责协调和分配资源.

- 文件系统
- 网络设备
- 各类 IO 管理

## 操作系统架构

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
|    ![\|200](../../attach/Pasted%20image%2020240429171126.avif)            |  ![安卓系统五层架构\|200](../../attach/Pasted%20image%2020230620164006.avif)            |



***


## System Startup 

`init` initialization code from kernel 

1. 主板加电, 硬件自检
2. 引导加载程序 (BootLoader): UEFI/BIOS
3. 加载内核: 启动内存管理, 驱动, CPU 调度
4. 执行内核的初始化进程 (init): 搭载根文件系统, 配置网络和其他资源

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

1. 块设备 (block device), 按块缓存传输.
2. 字符设备 (char device), 字节流传输.

|              | 字符设备               | 块设备             |
| ------------ | ---------------------- | ------------------ |
| 缓存机制     | 不缓存                 | 系统缓存           |
| 随机访问     | 不支持, 仅顺序访问     | 支持, 任意位置读写 |
| 常见设备类型 | 键盘, 鼠标 `/dev/input/mice/`, 串口 `/dev/ttyS0`, 终端 | 硬盘 `/dev/sda`, SSD, USB `/dev/sdb`                  |

常见文件类型有:
1. file (f)
2. directory (d)
3. symlink (l)
4. executable (x)
5. empty (e)
6. scket (s)
7. pipe (p)
8. char-device (c)
9. block-device (b)

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
