## KConfig 选项

- KCOV: 
	- Kernel hacking / Kernel Testing and Coverage / Code Coverage for fuzzing: on
	- Kernel hacking / Kernel Testing and Coverage / Code Coverage for fuzzing / Instrument all code by default: off
- KASAN:
	- Kernel hacking / Compile-time checks and compiler options / Debug information / Generate DWARF Version 4 debuginfo 
	- Kernel hacking / Memory Debugging / KASAN: dynamic memory safety error detector: on 
	- Kernel hacking / Memory Debugging / KASAN: dynamic memory safety error detector / Instrumentation type / Inline Instrumentation 
- Detect Soft/Hard Lockups: Kernle hacking / Debug Ops, Lockups and Hangs 
- Detect Memory Leak: Kernel hacking / Memory Debugging / Kernel memory leak detector: on

## DebugFS

- `/sys/kernel/debug/kfence/stats`
- `/sys/kernel/debug/kfence/objects`

## KFENCE 

[Kernel Electric-Fence](https://docs.kernel.org/dev-tools/kfence.html). 减少采样频率, 牺牲准确性来换取效率, 比 KASAN 更轻量. 支持检测堆内存错误 Out-of-Bounds, Use-after-Free, Double Frees. KFENCE 是受 [GWP-ASAN](llvm%20hardened.md) 启发的工具.

在每次采样中, 默认一次 slab/slub alllocation 会被 KFENCE 拦截, 改为从 KFENCE Object Pool 中开辟内存.  

### static key 

```c
if (unlikely(kfence_enabled))
    do_something();
```

在 `key = false` 时, 指令替换为 `nop`; 在 `key = true` 时, 指令被 **patching** 为分支指令. 这样在关键路径 (fast path) 中, 分支就可以几乎无开销.

KConfig 开启 `CONFIG_KFENCE_STATIC_KEYS=y`.

### Out-of-Bounds 检测

页分配规则: KFence Object 分配在独立的页, 该页的前后页也受 KFence 控制, 称为 *Guard Pages*. Guard Page 被标记为 protected, 因此访问时会触发 Page Fault, 该 Fault 会被 KFence 处理, 并报告 OOB. 

在 KFence Object 所在的页内, 不可用区域被分配为 RedZone.

### Use-after-Free 检测

KFence Object 被释放后, 该页会被标记为 protected, 然后并不立即释放而是挂到 KFence freelist 中. 对 freelist 中内存的访问会报告 UAF.

当 KFence Object Pool 的使用率达到 75% 以上, KFence 会限制采样 "类似的" Object. 判断方法是, 限制由*相似调用栈*申请的 Object, 让采样点更多样. 否则, 一些长寿命的 pagecache 会快速将 KFence Object Pool 占满.

## KCOV 

## KASAN 

## KCSAN KTSAN

## ftrace

ftrace 用于追踪内核函数调用:
- 用户态: 用 ltrace, 一般用于追踪 libc 函数
- 系统调用追踪: 用 strace 
- 内核追踪: ftrace / kprobe 
- 硬件: perf 

### 配置 ftrace 

`make menuconfig`

``` 
CONFIG_DEBUG_FS=y  
CONFIG_FTRACE=y 
CONFIG_FUNCTION_TRACER=y
CONFIG_HAVE_FUNCTION_GRAPH_TRACER=n  # 内核支持的图形显示
CONFIG_FCUNTION_GRAPH_TRACER=n       # 以图形显示的方式展示函数追踪
CONFIG_FTRACE_SYSCALLS=y             # 追踪系统调用
```

如果不存在 `/sys/kernel/debug` 路径, 则执行:
```bash
mount -t debugfs none /sys/kernel/debug
```

### 使用

ftrace 在 `/sys/kernel/debug/tracing` 下提供接口:
- `trace`: 里面显示所有的追踪内容
- `tracing_on`: 开关追踪, `echo 0/1 > tracing_on`
- `current_tracer`: tracer 类型
- `available_tracers`: 可用 tracer 类型:
	- nop 
	- function: ftracer
	- function_graph: graph ftracer 
	- `set_ftrace_filter`: 过滤器, `echo :mod:bluetooth/n:mod:rfcomm > set_ftrace_fileter` 开启对蓝牙模块的追踪

## 

## 参考

https://docs.kernel.org/dev-tools/kfence.html

