## System Info

- 系统网络相关工具见 [bash/network](network.md)
- 系统存储相关工具见 [file system](file%20system.md)

### `/proc` Info

详见: [proc](../../System/File%20System/proc.md)

### `uptime`, `w`

展示已开机时间, 用 `w` 也可, `w` 还可以查看当前登录用户.

### `uname`

Shows kernel information.  

```bash
uname -a # Unix/Kernel 
# or
lsb_release -a # Linux Release 
```

### `sysstat`

`sysstat` 包是收集系统性能和使用情况的工具集, 包括 `mpstat`, `iostat`, `vmstat`, `sar`

`mpstat`, multiprocessor statistics, 报告 CPU 在用户态/内核态/iowait(等待I/O时间)/IDLE(空闲) 状态下所花费时间的百分比.

`iostat`, I/O statistics, 报告 各个设备传输速率, 每次 I/O 操作平均大小, 每秒进行的 I/O 操作数.

`vmstat`, virtual memory statistics, 报告关于虚拟内存/进程/CPU活动等信息.

```shell
$ vmstat 5 # 5s 刷新一次
procs -----------memory---------- ---swap-- -----io---- -system-- ------cpu-----
 r  b   swpd   free   buff  cache   si   so    bi    bo   in   cs us sy id wa st
 0  0   0  14900552  58160 397884   0    0     7     2    2   10  0  0 100  0  0
 0  0   0  14900552  58160 397884   0    0     0     0    3   36  0  0 100  0  0
```

| r                        | b                        | swpd            | free                 | buff       | cache          |
| ------------------------ | ------------------------ | --------------- | -------------------- | ---------- | -------------- |
| 运行队列                 | 等待 I/O 的进程数        | 虚拟内存量      | 空闲内存量           | 缓冲内存量 | 缓存内存量     |
| si                       | so                       | bi              | bo                   | in         | cs             |
| 从磁盘交换到内存数据量/s | 从内存交换到磁盘数据量/s | 读入块数        | 写出块数             | 中断数/s   | 上下文切换数/s |
| us                       | sy                       | id              | wa                   | st         |                |
| 用户态CPU时间占比        | 系统态CPU时间占比        | 空闲CPU时间占比 | 等待I/O的CPU时间占比 |   被偷取的CPU时间(如虚拟机使用的CPU时间)        |                |

### `dstat`, `glances`

`dstat` 可以视作 `sysstat` 的简易现代替代品. 

更深度系统信息则可以使用 [glances](https://github.com/nicolargo/glances), 会展示更多系统级数据.

### `free`

`free` 检查主存空间

### `lsxxx` series

查看硬件信息的一系列工具:
- `lsblk` list block devices, 如硬盘驱动器, 固态驱动器和usb等的信息.
- `lshw` list headware, 展示详细系统硬件信息.
- `lscpu` list cpu, 展示 CPU 架构信息
- `lspci` list PCI, 展示PCI总线上设备, 如显卡/网卡/声卡
- `lsusb` 展示 USB 设备的信息.
- `dmidecode` 解析系统 DMI (桌面管理接口), 提供主板/BIOS/处理器/内存信息.

## Screen

分屏软件: `screen`, `tmux`, `byobu`, `dtach`

## Shutdown

`shutdown`

`halt` 实际上调用了`shutdown -h`

重启: `reboot`

## Time

### `dateutils`

[`dateutils`](http://www.fresse.org/dateutils/) 中 `dateadd`、`datediff`、`strptime` 系列工具