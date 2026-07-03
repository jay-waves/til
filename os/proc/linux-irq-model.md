## 系统调用

系统调用过程详见 [syscall](../libc/syscall.md), 用于从用户态切换为内核态, 执行内核功能.

## 硬中断（上半区）

硬中断处理函数, 可能来自定时器, 外部设备等. 它会抢占 CPU, 有如下特征:
1. 直接在当前内核栈上执行, 但不影响当前 task 的上下文状态
2. 不参与调度. 意味着不允许睡眠或阻塞, 也禁用可能触发中断的函数.
3. **异步执行, 并且允许重入.**
4. 一般不执行复杂逻辑, 只作为中断上半区.

为了避免中断上下文执行过长时间任务, 将中断处理程序分为上下两个半区. *中断上半区 (Top Half)* 在中断触发时立即执行, 执行时间短, 以最小化中断禁用时间. *中断下半区 (Bottom Half, BH)* 在中断触发后, 可能以较低优先级被延迟执行, 用于执行较耗时并且无严格时间限制的操作.  

## 中断下半区

中断下半区的执行机制比较多，如 SoftIRQ, Workqueue, kworker 等都可以：

| 执行器       | 执行上下文            | 可睡眠 | 普通内核调度器 | 描述                   |
| ------------ | --------------------- | ------------ | -------------- | ---------------------- |
| Process      | 进程                  | 是           | 可调度         |                        |
| KThread      | 进程                  | 是           | 可调度         |                        |
| Hard IRQ     | 硬中断                | 禁止         | 不可调度       | 中断上半区             |
| Threaded IRQ | 进程                  | 是           | 可调度         | 普通设备中断处理           |
| SoftIRQ      | 软中断                | 禁止         | 不可调度       | 网络等高频设备         |
| ~~Tasklet~~  | 软中断 (基于 SoftIRQ) | 禁止         | 不可调度       | 废弃                   |
| Workqueue    | 基于 kworker          | 是           | 可调度         | 可延迟或阻塞的异步任务 |

主要需要考虑：**是否需要睡眠？是否需要低延迟？**

不可睡眠，是指**禁止调用可能触发调度的函数**, 防止该中断任务被内核 schduler 抢占并触发调度。这些中断任务可能要求**原子性或锁依赖**，被抢占后可能无法恢复或死锁。

明确不会触发调度的函数比如:
- 原子内存分配 `kmalloc(GFP_ATOMIC)`
- 自旋锁 `spin_xxx()`
- 原子操作 `atomic_inc()`, ...
- 延迟执行: `tasklet_shcedule(), raise_softirq()`

可能触发调度的函数：
* 直接睡眠或阻塞的 `wait_xxx(), mutex_lock()`
* 可能内存分配 `kmalloc`, `vmalloc`
* 可能触发缺页中断的 `copy_to_usr()` 

一定要触发睡眠的中断处理函数，可以放在 `Threaded IRQ` 或者 `Workqueues` 延迟执行。

### 处理来自硬件的中断

内核延迟执行的流水线:

IRQ Handler -> SoftIRQ -> Workqueue -> Kernel Thread

1. 硬件设备触发 IRQ，由硬件 IRQ Controller （如 APIC）告知 CPU 
2. CPU 处理 IRQ  信号
	- CPU 触发硬件中断，抢占执行机会，搁置原执行上下文。
	- 大部分架构中，CPU 把 PC 跳到 Vector Table 硬件向量表来执行 IRQ 调度
3. 中断上半区：执行非阻塞、最小化任务，将重型中断任务延迟
4. 中断下半区：延迟执行中断任务，通常可由内核统一调度
5. 中断处理完后，返回原执行上下文。

## 延迟任务

### SoftIRQ 

ksoftirqd (kernel thread)

延迟执行的中断处理程序, 禁止睡眠或阻塞. 

> Tasklet 则是对 SoftIRQ 的封装，不过已经废弃，被 Workqueue 和 THreaded IRQ 取代。
> 类似的，内核早期（<2.4 版本）的 Bottom Half 机制也被废弃。

### Workqueue

将任务交付给内核线程 `kworker`, 适合可能会阻塞和睡眠的工作, 直接工作在进程上下文.

```c
struct work_struct {
	atomic_long_t data;
	structlist_head entry;
	work_func_t func;
	...
};

INIT_WORK(&my_work, my_func);
schedule_work(&my_work);
```

workqueue 的类型非常多, 如多线程 wq, 高优先级 wq, 内存回收 wq.

### Threaded IRQ 

让中断处理函数 (IRQ handler) 直接运行在内核线程中. 

#### NAPI Threads

### Timer 

执行 `TIMER_SOFTIRQ` 上下文, 不能睡眠.

### RCU Callbacks 

