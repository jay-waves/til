## Linux 内核源码目录

- `arch` 硬件架构
	- `i386` interrupt: (早期 intel cpu)
		- `arch/i386/kernel/irq.c`
		- `include/asm-i386/irq.h`
	- `x86`
		- `arch/x86/entry`
	- `arm`
- `block` 块设备驱动程序 IO 调度
- `crypto` 密码算法, 压缩算法等
- `documentation`
- `drivers` 设备驱动程序, 如 `char, block, net, mtd`
	- PCI Pseudo Driver: `drivers/pci/pci.c`, `include/linux/pci.h`
- `fs` 各文件系统
	- `ext4`
	- `vfat`
	- `fs/*`, `include/linux/fs.h`: vfs
- `include` 内核 API 级头文件. 
	- `include/linux/syscalls.h`
	- `include/linux/cred.h`: credentials like UID/GID, keyrings, capability sets
	- `include/linux/sched.h`
- `init` 内核初始化代码. 如 `init/main.c`
- `ipc` 进程间通信代码
- `irq` 中断服务
- `kernel` 内核核心代码. 平台相关的则放在各个 `arch/*/kernel` 下
	- `kernel/sys.c`: system calls
	- `kernel/sched.c`: scheduler
	- `kernel/fork.c`
	- `kernel/nsproxy.c`
	- `kernel/cgroup`: cpu, memory, io constraints and accounting
- `lib` 库文件代码. 平台相关在 `arch/*/lib`
- `mm` 内存管理. 平台相关放在 `arch/*/mm`
	- `mm/memory.c`: 缺页错误处理
	- `mm/page_alloc.c`
	- `mm/filema.c`: 内存映射和页缓存
	- `mm/buffer.c`: buffer cache
	- `mm/swap_stat.c, mm/swapfile.c`: swap cache
	- `mm/mmap.c`
- `net` 网络
	- BSD Socket: `net/socket.c`
	- IPv4: `net/ipv4/af_inet.c`
	- TCP/IP: `net/ipv4`
	- Net Drivers: `dirvers/net`
- `scripts` 用于配置内核的脚本
- `sound` 音视频驱动
- `security`: LSM & seccomp, like AppArmor / SELinux
- `usr`

## Linux 编译结构

- `Makefile`: 顶层 Makefile, 
	1. 读取和配置 .config 
	2. 读取内核版本 `/Linux/version.h`
	3. 读取 `arch/$(ARCH)/Makefile$` 中的架构相关配置
	4. 递归进各个子目录, 执行编译
	5. 执行 `scripts/head-object-list.txt, link-vmlinux.sh`, 链接最终镜像 vmlinux 
- `.config`: 配置文件
- `arch/$(SRCARCH)/Makefile$`: 架构相关 Makefile
- `scripts/Makefile.*` 
- `Kbuild + Makefiles` 每个子目录的 makefile. 注意 KBuild Makefile 的语法和普通 Makefile 不同, 并且和 KConfig 集成. 

```kbuild 
obj-$(CONFIG_FOO) += foo.o

obj-y += subdir/

ccflags-$(CONFIG_XXX_DEBUG) += -XXX_DEBUG -Od
```

`CONFIG_FOO` 在 `.config` 中配置, 有 `y/m/n` 三种选项. 

- 对象 `obj-y` 会被脚本链接入最终镜像 `vmlinux`.
- 对象 `obj-m` 会被编译为独立的内核模块.
- Makefile 只负责同目录下的文件, 所有子文件要显式添加. 