## 何为 C 标准?

C 语言诞生于 AT&T 的贝尔实验室, 但功能不完善. 1983 年美国国家标准协会 (ANSI) 
成立 C 语言标准委员会, 于 1989 年建立第一个完整的 C 语言标准, 称为 ANSI C89, 
其中包括了标准函数库[^7].

| 操作系统                   | 编译环境   | C 标准实现    | C++ 标准实现 | 解释                                                                      |
| ------------------------------| ---------- | ------------- | ------------ | ------------------------------------------------------------------------- |
| GNU/Linux                   | GCC        | glibc         | libstdc++    |                                                                           |
| GNU/Linux                         | Clang/LLVM | <nobr>llvm-libc[^2]</nobr> | libc++       | 较旧版本的 LLVM 仍使用 glibc, 而 libc++ 是 LLVM 独立实现的现代 C++ 标准库 |
| Alpine Linux                   |              | musl libc     |              | Alpine 发行版使用 musl libc                                               |
| BSD (FreeBSD, NetBSD, OpenBSD) |              | BSD libc      |              |                                                                           |
| macOS, iOS                     |   Clang/LLVM | apple libc    | libc++       | macOS 前身基于 BSD, 因而 Apple libc 基于 BSD libc                         |
| Windows                        |   Clang/LLVM | MSCVrt [^3]   | MSVC++       | 一套标准实现就支持*多平台*是 LLVM 的设计目标                              |
| Windows                        |    GCC/MinGW  | MSVCrt        | MSVC++       | 借助 MinGW 模拟 GNU[^1]                                                   |
| Windows                        |  MSVC[^6]       | MSVCrt        | MSVC++       | Microsoft Visual C, 原生库和集成环境                                      |
| 嵌入式系统                     |               | newlib        |              |                                                                           |

## 系统调用

因为 C 语言是系统级语言, 除了通用标准库 libc 外, 编译器还需要实现大量系统调用. 如在类 Unix 系统上 (BSD, Linux, MacOS), 常使用 POSIX 标准定义的 API; 在 Windows 上, 则使用 Windows API.

| 系统调用标准                                    | 维护组织           | 解释                                       | 平台支持      |
| ------------------------------------------- | ------------------ | ------------------------------------------ | ------------- |
| POSIX (POSIX Operating System API)                                     | IEEE               |                                            | Unix-like     |
| Windows API                                 | Microsoft          |                                            | Windows       |
| LSB (Linux Standard Base)                   | Linux 基金会       | 提高 Linux 不同发行版之间的兼容性          | Linux         |
| SVID (System V Interface Definition)        | AT&T, Unix SystemV | 不常见, 仅兼容[^5]            | Unix SystemV |
| RTOS (Real-teim Operating System Standards) |                    | 在嵌入式系统中, 用于提供更低延迟和更高确定性 |               |
| XPG, SUS                                     | The Open Group     |   不常见                                         | Unix[^4]          |

**不同的平台和工具链, 会导致不同的ABI/API. 和软件的兼容性休戚相关.**

例子: llvm 预编译安装包命名为: `llvm-18.1.8-x86_64-pc-windows-msvc.tar.xz`, 软件名及版本为 `llvm-18.1.8`, CPU指令集及字长为 `x86_64`, 平台为 `pc`, 系统为 `windows`, 编译工具链为 `msvc`.

其他, `llvm-18.1.8-aarch64-linux-gnu`, `llvm-18.1.8-armv7a-linux-gnueabihf`, `llvm-18.1.1-powerpc64le-linux-rhel-8.8`. powerpc64le -> PowerPC 64-bit Little Endian, linux-rhel -> Red Hat Enterprise Linux 8.8

### 系统调用头文件

SVID 系统调用头文件: Unix 系统最早由 AT&T 开发, 后续分支有 BSD 和 System V. System V 接口标准为 SVID, 也有其他不同接口标准.

```c
#include <sys/ipc.h>   // 进程间通信
#include <sys/shm.h>   // 共享内存
#include <sys/sem.h>   // 信号量
#include <sys/msg.h>   // 消息队列
```

POSIX 系统调用头文件: IEEE 在 1988 年制定了 POSIX 标准用于统一 UNIX 系统的接口, 主要是基于 System V 和 BSD 接口.

```c
#include <unistd.h>    // 系统调用, 进程管理, 文件输入输出
#include <fcntl.h>     // 文件控制
#include <sys/types.h>
#include <sys/stat.h>  // 文件类型
#include <pthread.h>   // 线程
#include <signal.h>    // 错误处理
#include <errno.h>     // 错误码
```

LSB 系统调用头文件: Linux 参考 POSIX 标准, 同时也为了兼容性, 大量引入 System V 接口.

```c
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/syscall.h> // 直接系统调用
```

Windows API:

```c
#include <windows.h>    // 核心系统调用
#include <process.h>    // 进程管理
#include <io.h>         // 低级文件 I/O
#include <direct.h>     // 目录操作
#include <winsock2.h>   // 网络编程
#include <synchapi.h>   // 线程同步
#include <errhandlingapi.h> // 错误处理
```

如果想在 Windows 上使用 POSIX 系统调用, 可以使用 Cygwin 或 MinGW 等提供 POSIX 接口的兼容层; 或者使用 Boost, Qt 等上层库. 对于文件, 线程管理等功能性接口, 使用 C/C++ 语言标准库也可以屏蔽平台的差异.

## 参考

https://users.cs.cf.ac.uk/Dave.Marshall/C/. Dave Marshall. 1999-03.


[^4]: SUS 和 XPG 都是针对**正式 UNIX** 系统的, 而不是类 Unix 系统. 

[^5]: System V 是 AT&T 开发的 UNIX 版本, 其 System V IPC (跨进程通信) 和 SystemV Init (系统进程启动和管理) 接口影响广泛. 但现在影响力已较弱, 如 SysV IPC 被 POSIX IPC 取代, SysV Init 被 systemd 取代.

[^6]: MSVC: MicroSoft Visual C/C++ 集成开发环境, 现已集成到 MiscroSoft Visual Studio 中. 相关编译器工具可以用 Developer Command Prompt for VS 直接访问.

[^1]: 注意: MinGW  (Minimalist GNU for Windows) 并不能完全移植 Unix-like 系统上的 C 程序到 Windows. MinGW 仍使用 Windows 系统 API, 而仅**部分**兼容 POSIX API, 其旨在提供类 GNU 的开发链条, 方便开箱即用, 但是不适合大型程序. 如果需要兼容完整 POSIX 环境, 即提供完整 POSIX API 到 Windows API 的翻译, 可以用 **Cygwin**.

[^2]: llvm-libc 正在开发中, 不稳定; llvm-lld 也不稳定. 所以现在在 windows 上用不了.

[^3]: MSVCrt (microsoft visual c runtime) 指 `msvcrt.dll`, 是 Windwos 系统对 libc 的实现. 在 Windows10 之后, 引入了 UCRT 的现代版本 libc 实现, 提供了对现代 C 标准更好的支持. 都不开源...

[^7]: 具体函数库内容详见 [C/StandardLib/ReadMe](../../Language/C/标准库/ReadMe.md) 语言标准库, libc++ 指 C++ 标准库. 语言委员会提供一套标准库函数标准, 但不规定底层究竟如何实现 (比如在 linux 和 windows 上, `printf()` 可能会调用不同的系统调用来达成功能), 所以不同编译器和操作系统都会有独立的实现.