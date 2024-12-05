测试软件性能, 也叫 Benchmarking.

最基础工具: `time`

```bash
time xxx.a 
```

hyperfine, 更精准, 有预热过程等.

```bash
hyperfine --runs 5 --warmup 3 'sleep 0.3'
```
