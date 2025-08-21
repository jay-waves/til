## 系统调用

系统调用过程详见 [系统调用](../../Compiler/运行时/系统调用.md), 用于从用户态切换为内核态, 执行内核功能.

## 硬中断

硬件中断处理函数, 不参与调度, 直接抢占 CPU. 不允许睡眠或阻塞, 也禁止使用可能触发调度的函数 (mutex, kmalloc) 等. 复杂逻辑应推迟到 SoftIRQ. 如果一定需要睡眠或阻塞, 中断任务应被交付给 workqueues, 后续被内核线程 kworker 执行.

内核延迟执行的流水线:

IRQ Handler -> SoftIRQ -> Workqueue -> Kernel Tread

内核延迟执行的流水线:

IRQ Handler -> SoftIRQ -> Workqueue -> Kernel Tread

| 执行器    | 上下文         | 是否可睡眠 | 调度         | 内存                       |  是否可抢占 |
| --------- | -------------- | ---------- | ------------ | ---------------------- | ---- |
| Process   | 系统调用    | 是         | 内核调度     | 独立用户态虚拟地址空间 |   是   |
| KThread   | 内核态守护进程        | 是         | 内核调度     | 仅内核地址空间         |  是    |
| IRQ       | 硬件中断处理     | 禁止       | 抢占执行     | GFP_ATOMIC             |  否    |
| SoftIRQ   | 网络, 定时器等   | 禁止       | 内核触发     | GFP_ATOMIC             |   否   |
| Tasklet   |  | 禁止       | 串行化软中断 | GFP_ATOMIC             |    否  |
| Workqueue |                | 是         |              |                        |  是    |

在 IRQ, SoftIRQ, Tasklet 执行, **禁止调用可能触发调度的函数**, 防止当前 CPU 放弃执行切换到其他 task. 比如阻塞等待类 `wait_*(), down(), mutex_lock()`, 可能导致睡眠的内存分配 `kmalloc(GFP_KERNEL), vmalloc()`, 可能触发缺页中断的 `copy_to_usr(), vfs_write()` 等. **如果导致内核调度到其他上下文, IRQ 等上下文是无法再恢复的, 导致系统栈状态损坏, 进而崩溃.**

明确不会触发调度的函数有:
- 原子内存分配 `kmalloc(GFP_ATOMIC)`
- 自旋锁 `spin_xxx()`
- 原子操作 `atomic_inc()`, ...
- 延迟执行: `tasklet_shcedule(), raise_softirq()`

### 硬中断下半区

Linux 内核中, 为了避免中断上下文执行过长时间任务, 将中断处理程序分为上下两个半区. *中断上半区 (Top Half)* 在中断触发时立即执行, 执行时间短, 以最小化中断禁用时间. *中断下半区 (Bottom Half, BH)* 在中断触发后, 可能以较低优先级被延迟执行, 用于执行较耗时并且无严格时间限制的操作.

中断上下文有严格限制: 禁用抢占和调度, 不能睡眠, 不能调度阻塞函数. 不适合执行复杂逻辑, 也不适合长时间占用, 因此推荐将非紧急任务推迟到下半区, 由内核适时调度执行. 在早期内核版本 (2.4 以下), 使用单一 BH 机制, 后被 SoftIRQ (软中断), Tasklet, Workqueue 等机制取代.

### SoftIRQ 

ksoftirqd (kernel thread)

延迟执行的中断处理程序, 禁止睡眠或阻塞. Tasklet 是对 SoftIRQ 的封装, 但保证不会在单个 CPU 核心上并发, 并且是串行的.

#### Tasklet 

## 延迟任务

### Workqueue

可以睡眠, 用 kworker.

### Timer 

执行 `TIMER_SOFTIRQ` 上下文, 不能睡眠.

### RCU Callbacks 