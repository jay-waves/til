编译时 Config 选项:
- KCOV: 
	- Kernel hacking / Kernel Testing and Coverage / Code Coverage for fuzzing: on
	- Kernel hacking / Kernel Testing and Coverage / Code Coverage for fuzzing / Instrument all code by default: off
- KASAN:
	- Kernel hacking / Compile-time checks and compiler options / Debug information / Generate DWARF Version 4 debuginfo 
	- Kernel hacking / Memory Debugging / KASAN: dynamic memory safety error detector: on 
	- Kernel hacking / Memory Debugging / KASAN: dynamic memory safety error detector / Instrumentation type / Inline Instrumentation 
- Detect Soft/Hard Lockups: Kernle hacking / Debug Ops, Lockups and Hangs 
- Detect Memory Leak: Kernel hacking / Memory Debugging / Kernel memory leak detector: on

## KCOV 

## KASAN 

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