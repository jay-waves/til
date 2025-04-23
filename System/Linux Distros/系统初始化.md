## Linux 系统初始化

1. 主板加电, 硬件自检. 
2. 每个 SoC 核心内嵌了 bootrom, 开始执行.
3. 只有 CPU0 核心的 bootrom 会加载 *引导程序 (BootLoader)*, 如: URFI/BIOS
4. 其他 CPU 核心进入等待状态.
5. CPU0 上的 BootLoader 加载内核, 
6. 内核启动阶段, CPU0 会触发中断唤醒其他 CPU 核心, 全部进入内核启动状态.
7. CPU0 的内核调用用户空间初始化程序 init. 由它派生其他进程, 如: 搭载根文件系统, 配置网络资源.