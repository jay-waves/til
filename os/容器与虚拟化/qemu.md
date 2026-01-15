Qemu 用于仿真硬件, 开销不小, 一般只用来 I/O 虚拟化. 硬件支持的 KVM 虚拟化技术, 则负责更繁琐的 CPU 和 内存虚拟化.

```

Target
  |
  v
 Qemu (Guest Code --> TCG --> Host Code)
  |
  v
 Host
```

Qemu 的源码主要包含几个部分:
- `vl.c` 主循环, 包括虚拟机环境初始化和 CPU 的执行.  --> `main_loop()`
- `cpus.c` 分时运行 CPU 核 --> `qemu_main_loop_start()`
- `translate-all.c` --> `cpu_gen_code()`
- `<target-arch>/translate.c` 将 guest 代码翻译为不同架构的 TCG 操作码. 
- `<target-arch>/cpu.h` --> `struct CPUState;`
- `tcg/tcg.c` 主要的 TCG 代码. --> `tcg_gen_code()`
- `tcg/<arch>/tcg-target.c` 将 TCG 代码转化为对应架构的主机代码
- `cpu-exec.c` 寻找下一个二进制翻译代码块. --> `cpu_exec()`

## Unicorn 

Unicorn 是 Qemu 的一个轻量级分支, 主要用于安全分析用途. Unicorn 支持 CPU 仿真 (纯指令级执行), 不支持设备 / 外设等仿真, 但是提供了高层次的 Tracing API, 用于监视和分析二进制程序状态. 在 Qemu 的早期版本, 不支持 QEMU Plugins, 因此修改 QEMU TCG 功能后, 需要重新编译 QEMU, 比较不方便.

Unicorn 不能自动模拟 UART/TIMER 等常见外设, MMIO 也需要自己通过 `hook_mmio_read/write()` 来手动模拟行为. 也不支持模拟中断响应, 需要用 `timer_interrupt(mu)` 来手动模拟.

## QEMU Plugins 

通过 TCG 来编写插件, 是只读的. 较新的插件版本允许动态加载, 但是不保证 API 向后兼容. 

```
configure --enbale-plugins 

qemu-xxxx   -plugin xxxx/plugin/libhotblocks.so
```

导出所需的 qemu plugin api 版本, QEMU 会检查.
```c
QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;
```

插件的生命周期:

1. 插件装载时, 调用 `qemu_plugin_install()` 
2. 用 `atexit()` 来注册退出回调, 用于保存信息等.
3. `qemu_plugin_xxx()` 系列函数.

### 

```c
#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT 
int qemu_plugin_install(qemu_plugin_id_t id, const qemu_info_t *info, int argc, char **argv) {
	qemu_plugin_reguister_vcpu_tb_trans_cb(id, xxxx);
	return 0;
}

/*
	TB translation callback. 
*/
```

### Qemu Plugin API 

...

寄存器读取能力见: https://patchew.org/QEMU/20230803112551.14803-1-m.tyutin@yadro.com/