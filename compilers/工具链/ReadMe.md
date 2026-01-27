现代项目构建流程:
```
元构建系统 (autoconf, cmake, bazel, gn) 

		--> 构建系统 (make, ninja, VS) 
		
				--> 编译器 (msvc, gcc, clang)
```

现代软件项目有复杂的源码构成以及编译顺序, 需要的编译器命令庞杂. 构建系统 (make) 将多个编译器命令组成一个脚本 (Makefile), 来自动执行这些命令. 
但是脚本命令和平台是强相关的, 缺乏处理库依赖和持续集成 (CI) 等功能. 元构建系统 (cmake) 负责, 自动生成平台相关的 Makefile.

### Triplets 

Triplets 就是指编译过程的"环境", 也称为"[平台](../../os/syscalls/libc.md)". 执行当前编译过程的称为*宿主机平台*, 编译产物的实际运行环境称为*目标平台*, 两者可以不同. 比如我在 x86-windows 上编译可以跑在 arm-linux 上的程序. Triplets 直译为三元组, 但实际上现代编译系统需要四元组来区分不同的平台. 比如, 同样使用 Arm [芯片指令集架构](../指令集架构/ReadMe.md) 和 Linux 内核的 安卓系统 `aarch64-unknown-linux-android` 与 Ubuntu 系统 `aarch64-unknown-linux-gnu` 的程序就不能互通.

Triplets 由以下部分构成
- CPU: x86 (i686, i386), amd64 (x86_64), arm, aarch64 (arm64), riscv64, wasm32
- Vendor: 可以省略. 如 `pc, unknown, apple`.
- OS Kernel: windows, linux, macOS (apple), freebsd
- OS ABI (SDK, C Runtime...): gnu, gnueabihf (硬件浮点支持), musl, android, mingw32, msvc, darwin

## 常见元构建系统


### Meson

Meson, 2012. 

- 脚本语言类似 Python.
- 配置文件为 `meson.build`
- 构建系统生成器仅为 Ninja, 不支持 VS, Make
- 自带依赖管理工具和数据库 WrapDB. 不支持 VCPKG