![linuxperf](../../appx/Linux%20Perf%20Tools.avif)

核心工具：
* **`htop`** 任务管理器
* `mpstat` 多核利用率和软中断
* `pidstat` 精确统计 CPU、IO、上下文切换
* **`perf record/stat/top`** 分析 CPU HotSpot、Cache Miss、上下文切换
* `free`  查看内存
* **`vmstat`** 查看内存 SWAP、IO Wait 行为
* **`iostat, iotop`** 查看磁盘 IO 瓶颈
* `fio` 磁盘 IO 的主动压测（benchmark）工具 
* `blktrace` 内核块设备延迟
* **`ss`** 查询网络链接状态
* `tcpdump` 抓包
* `strace` 系统调用分析，排查程序阻塞、颠簸等
* `lsof` 排查文件描述符泄漏
* `bpftrace` 利用 eBPF 的动态追踪：syscall 统计、IO 统计、延迟

## 高频问题排查

### 1. IO 吞吐下降

### 2. CPU 满载但程序慢

### 3. perf 定位热点

详见 [进程调试-perf](进程调试.md)

### 4. benchmark 如何保证公平性？

### 如何判断瓶颈是 CPU 还是 IO？

`perf stat` ：
* IPC 低，cache-miss 高，说明是 IO 瓶颈
* branch-miss 高，说明是分支预测问题
* context-switch 高，说明是调度问题
* spin-lock 高，说明在频繁锁竞争

### 多核扩展性差？核增加没有提升

参考我设计的[线程池](../../algo/concurrency/thread-pool.md)

### 如何设计一个