
bpftrace 是封装 eBPF 的一种上层 DSL，供方便地写脚本调用 eBPF 能力。最终程序被翻译为 eBPF 二进制码，
交给内核的 eBPF VM 执行。

| Type | |
|---|----|
| `tracepoint` | kernel static instru points |
| `usdt` | user-level static defined tracing |
| `kprobe` | kernel dynamic func instru |
| `kretprobe` | kernel dynamic func return instru |
| `uprobe` | User-level dynamic func instru |
| `uretprobe` | User-level dynamic func return instru |
| `software` | kernel software-based events |
| `hardware` | hardware countr-based instru |
| `watchpoint` | memory watchpoint events |
| `profile` | timed sampling across cpus |
| `interval` | timed reporting |
| `BEGIN` | start of bpftrace |
| `END` | End of bpftrace |

* dynamic: instruments in-flight without restarting it
* static: points are hard-coded and become a stable api.

| variable | |
| --- | --- |
| `@name` | global |
| `@name[key]` | hash |
| `@name[tid]` | thread-local | 
| `$name` | scratch |

| builtin variables | |
| ------- | ----- |
| `pid` | process id |
| `comm` | proc or command name |
| `nsecs` | ns current time |
| `kstack` | kernalt stack trace |
| `ustack` | user-level stack trace |
| `arg0..argN` | functino argumetns |
| `retval` | function return value |


## tracepiont

bpftrace 提供了相当多的内置工具。

![](https://www.brendangregg.com/blog/images/2019/bpftrace_tools_early2019.png)

文件打开时：

```c
tracepoint:syscalls:sys_enter_openat {
    printf("%s %s\n", comm, str(args.filename));
}
```

计数：

```c
tracepoint:raw_syscalls:sys_enter {
    @[comm] = count();
}
```

指定进程绘制指数分布直方图：

```c
tracepoint:syscalls:sys_exit_read /pid == 1865/ {
    @bytes = hist(args.ret);
}

```
绘制均匀直方图：

```ebpf
kretprobe:vfs_read {
    @bytes = lhist(retval, 0, 2000, 200);
}
```

统计 `read()` 耗时：

```c
kprobe:vfs_read {
    @start[tid] = nsecs;
}

kretprobe:vfs_read /@start[tid]/ {
    @ns[comm] = hist(nsecs - @start[tid]);
    delete(@start, tid);
}
```

统计调度相关调用：

```c
tracepoint:sched:sched* {
    @[probe] = count();
}

interval:s:5 {
    exit();
}
```

统计调度上下文

```c
tracepoint:sched:sched_switch {
    @[kstack] = count();
}
```

## 参考

[One-Liner Tutorial](https://bpftrace.org/tutorial-one-liners)

[A thorough introduction to bpftrace, Brendan Gregg]( https://www.brendangregg.com/blog/2019-08-19/bpftrace.html)

[bpftrace language](https://github.com/bpftrace/bpftrace/blob/master/docs/language.md)