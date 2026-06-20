
### `ldd`

列出程序所依赖的共享库文件 (.so)

```bash
$ ldd /usr/bin/clang
 linux-vdso.so.1 (0x00007fffc04c3000)
 libclang-cpp.so.14 => /lib/x86_64-linux-gnu/libclang-cpp.so.14 (0x00007f8e58795000)
 libLLVM-14.so.1 => /lib/x86_64-linux-gnu/libLLVM-14.so.1 (0x00007f8e51ec3000)
 libstdc++.so.6 => /lib/x86_64-linux-gnu/libstdc++.so.6 (0x00007f8e51c97000)
 libm.so.6 => /lib/x86_64-linux-gnu/libm.so.6 (0x00007f8e51bb0000)
 #  [库名] => [地址] (加载到的内存地址)
```

ldd 存在[严重安全漏洞](https://catonmat.net/ldd-arbitrary-code-execution), 确保目标程序是受信任文件.

### `strace`, `ltrace`

用于审查程序失败/挂起/崩溃的原因, 或者大致了解程序性能.  strace 用于跟踪系统调用, `ltrace` 用于跟踪程序对动态库函数的调用 (也包括标准库函数 `malloc`, `printf` 等)

- `-c`: 开启 profile 性能分析, `strace -c ./my_program`
- `-p`: 附加到一个运行中的进程, `strace -p 1234`



### `dmesg`

引导及系统错误信息

### `/proc`

`/proc` 是一种虚拟文件系统, 用于内核态和用户态的通信. 用户可以通过这些文件读取大量系统信息, 并通过修改参数改变系统行为. `/proc` 并不在磁盘中存在, 而是通过 VFS 在内存中注册自己, 当 VFS 使用 `open(), read()` 等系统调用访问它时, 调用内核信息创建文件.

- `/proc/cpuinfo` CPU 详细信息
- `/proc/meminfo` 内存使用情况
- `/proc/sys/` 内核各种参数和状态, 用于修改内核行为 (如 IP转发)
- `/proc/partitions` 系统所有分区
- `/proc/mouts` 当前挂载的文件系统
- `/proc/[pid]/cmdline` 启动进程时使用的命令行参数
- `/proc/[pid]/status` 进程状态信息
- `/proc/[pid]/cwd` 进程工作目录
- `/proc/[pid]/exe` 指向该进程所用可执行文件的符号链接
- `/proc/[pid]/fd/` 指向该进程打开的所有文件的符号链接
- `/proc/iomem` 物理内存布局
- `/proc/[pid]/maps` 查看虚拟内存布局

#### `sysctl` 

`sysctl` 工作在 `/proc/sys` 目录下, 通过读写文件来改变内核参数.

```bash
# 查看所有 sysctl 配置
sysctl -a
# 开启 ip 转发
sysctl -w net.ipv4.ip_forward=1
```

如果要持久化修改配置, 需要修改 `/etc/sysctl.conf`

```bash
sudo echo "net.ipv4.ip_forward=1" >> /etc/sysctl.conf
sysctl -p
```