kvm 是一个内核模块, 用于封装现代 CPU 提供的虚拟化功能 (比如 Intel VT-x, AMD-V). kvm 通常和全系统虚拟机配合使用 (如 QEMU), 通过 `/dev/kvm` 提供用内核线程模拟的 vCPUs. 

kvm 在 non-root 模式 (虚拟机内核) 下执行所有指令. 当需要执行特权指令时, 会调用 VMEXIT 进入内核态 (root mode). 如果指令涉及 CPU 和内存状态, 则直接在内核中执行; 如果涉及 I/O 等设备操作, 会将事件转发给 QEMU, 让其模拟外设行为. 

kvm+QEMU 仅支持同构虚拟化, 即虚拟机和宿主机指令架构相同. 否则, 需要使用 TCG+QEMU, 对指令和硬件上下文进行动态二进制翻译. kvm 不是 Unicorn 那样的任意代码段执行引擎, 而是需要完整的 vCPU 状态机.

QEMU-KVM 模式的分工如下:
- QEMU 
	- VM 配置和启动. 加载 BIOS, 内核, 虚拟外设
	- 虚拟外设
- KVM:
	- 调用 CPU 虚拟化功能, 仿真 vCPU. 
	- 内存管理:
		- 由 QEMU 通过 `mmap()` 申请一块内存, 通过 `ioctl(KVM_RUN)` 注册给 kvm.
		- kvm 管理 GPA (Guset Pysical Address) --> HVA (Host Virtual Address) 映射, 该映射直接通过硬件 MMU 翻译. 
	- 中断注入


