## BOOT 自举

1. 加载*片上 ROM* 中的一小部分程序到*片上 SRAM* 中, 并执行. 
2. 该程序(1), 初始化*片外 Flash* 接口, 将存储器上的头部程序加载到 *片上 SRAM*, 并执行.
3. 该程序(2), 负责初始化*片外 DDR* 内存, 然后将*片外 Flash* 上的剩余程序复制到*片外 DDR*, 并执行.
4. 该程序(3), 负责初始化硬件, 并引导*内核*的启动. (至此, 称为 "BOOT 的自举")
5. 内核负责挂载根文件系统.

### BIOS 

BIOS 依赖于硬盘 0 扇区 (MBR), 非常古老的固件引导机制.

### UEFI 

UEFI 是更现代的 BIOS 系统, 已经类似一个小型操作系统. 从文件系统中直接加载 `.efi` 文件, 从而引导各类硬件 EFI 驱动.

UEFI 的启动过程:

| 阶段 | 全程                         | 作用 |
| ---- | ---------------------------- | ---- |
| SEC  | Security Phase               | 安全验证.     |
| PEI  | Pre-EFI Initilization        | 内存初始化, 建立 PEI 系统服务     |
| DXE  | Driver Execution Environment | 加载和执行硬件驱动程序     |
| BDS  | Boot Device Selection        | 选择启动设备, 如从光盘启动     |
| TSL  | Transitional System Load     | 加载操作系统引导     |
| RT   | Runtime                      | 操作系统加载完成后, UEFI 仍有一些运行时功能     |
| AL   | System Recoverag or Shutdown                             | 系统的灾难恢复, 关机或重启    |

## 内核初始化

1. 主板加电, 硬件自检.
2. Soc 核心执行其内嵌的固件 (BIOS, UEFI, BootRom)
3. 只有 CPU0 核心的 bootrom 会加载 *引导程序 (BootLoader)*, 如: GRUB, U-Boot.
4. 其他 CPU 核心进入等待状态.
5. CPU0 上的 BootLoader 加载内核镜像 (附带一些启动参数). 
6. 内核镜像的启动点, 是和架构相关的汇编代码, 其将构建一个执行 C ABI 的环境. (如清空 .bss, 初始化栈, 禁用中断等).
7. 接着, CPU 0 执行第一个 C 函数: `x86_64_start_kernel()`. 该函数在执行完架构相关代码后, 调用 `start_kernel()`. 这是通用的内核启动点.
8. 由 `start_kernel()` 依次唤醒 (按严格顺序): allocator, scheduler, timer, interrupts, per-CPU areas, RCU, workqueue 等内核基础工具.
9. CPU0 继续执行, 启动更高层次子系统: blcok I/O, fs, drivers, net.
10. 最后, 执行 `rest_init()`, 由其创建第一个内核线程 `kthreadd`, 以及用户空间初始化程序 `/sbin/init`. 并挂载根文件系统.

```
firware (BIOS, UEFI, BootRom)
	--> BootLoader (GRUB, U-Boot)
		--> architecture-dependent kernel entry (arch/x86/head.S)
			--> x86_64_start_kernel() (arch/x86/kernel/head64.c)
				--> start_kernel() (init/main.c)
					--> parse boot params
					--> allocators, schedulers, interrupts, timers, SMP
					--> initcalls by level (core, arch, subsys)
					--> subsystems: driver, ent, fs
					--> rest_init()
						--> create kthreadd (PID2)
						--> kernel_init() 
							--> kernel_init_freeable()
								--> mount root filesystem (rootfs, initramfs)
								--> /sbin/init 
```

> UEFI. Unified Extensible Firmware Interface, 统一可扩展固件接口, 替代传统 BIOS 的启动系统. 
