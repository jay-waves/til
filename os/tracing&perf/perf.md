## `perf` 

指标：
* cycles 
* IPC 
* branch-miss 
* cache-miss
* context-switch
* spin-lock

```bash 
perf stat gcc hello.c 
perf stat -p 1468

perf top # 热点
perf record -g  # 记录函数调用图，结果保存在文件
perf report 
```

### cpu 



### sched

## benchmark

- `time`: 执行命令, 并统计 CPU 时间
- `timeout`: 执行命令, 超时后自动停止
- `hyperfine`

```
$ time ./lab2
./lab2  0.59s user 0.01s system 19% cpu 3.008 total
```


[optick](https://github.com/bombomby/optick)


更深层的系统分析: `stap` ([SystemTap](https://sourceware.org/systemtap/wiki)), [`perf`](https://en.wikipedia.org/wiki/Perf_(Linux)), 以及[`sysdig`](https://github.com/draios/sysdig)=strace+tcpdump+htop.
