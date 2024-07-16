> MSVC 和正常软件不同, 还包括大量编程环境. 推荐安装到 C 盘, 并且不要手动修改目录 (用微软给定 Installer).

## CL

Visual Studio 包含了整个微软的配套编程环境, 从中下载 Visual C/C++. 其中就包含了编译器 `cl.exe`, 链接器 `link.exe`, PE/COFF 查看器 `dumpbin.exe`.


- `/Zi`           生成完整调试信息
- `/EHsc`          提供异常处理, c++ 需要开启
- `/c` 只编译, 不链接
- `/Za` 禁用一些 MSVC 的 C/C++ 专有扩展, 使程序和标准 C/C++ 兼容. 等价于定义 `#define __STDC__`
- `/nologo`        不显示版权信息
- `/source-charset:utf-8`     源码文件字符编码格式
- `/execution-charset:utf-8`   执行文件字符编码格式
- `/Fe.\\build\\...exe`        执行文件 .exe 和链接文件 .ilk 输出路径
- `/Fo.\\target\\...exe`       目标文件 .obj 输出路径                

### dumpbin

类似 objdump, 查看可执行文件.

```sh
dumpbin /ALL hello.obj > hello.txt

dumpbin /SUMMARY hello.obj
```

## C CRT

以 Visual C++ 2005 为例, 标准库实现有:

| 文件名      | DLL              | 属性                   | 编译选项 | 预编译宏            |
| ----------- | ---------------- | ---------------------- | -------- | ------------------- |
| libcmt.lib  |                  | 多线程[^2], 静态链接       | /MT      | `_MT`               |
| msvcrt.lib  | msvcrt80.dll[^1] | 多线程, 动态链接       | /MD      | `_MT, _DLL`         |
| libcmtd.lib |                  | 多线程, 静态链接, 调试 | /MTd     | `_DEBUG,_MT`        |
| msvcrtd.lib | msvcr90.dll      | 多线程, 动态链接, 调试 | /MDd     | `_DEBUG, _MT, _DLL` |
| msvcmrt     | msvcm90.dll      | 托管/本地混合代码      | /clr         |                     |

[^2]: 自 MSVC8.0 (Visual C++ 2005) 之后, MSVC 不再提供静态链接单线程版的运行库. 微软认为改进后的多线程运行库, 在单线程模式下运行速度已经接近单线程版本. 此后默认版本为 libcmt.lib

MSVC 也提供的 C++ 标准库, 称为 C++ CRT. 

| 文件名       | DLL          | 属性                   | 编译选项 | 预编译宏     |
| ------------ | ------------ | ---------------------- | -------- | ---------- |
| libcpmt.lib  |              | 多线程, 静态链接       | /MT      | `_MT`        |
| msvcprt.lib  | msvcp90.dll  | 多线程, 动态链接       | /MD      | `_MT, _DLL`  |
| libcpmtd.lib |              | 多线程, 静态链接, 调试 | /MTd     | `_DEBUG, _MT` |
| msvcprtd.lib | msvcp90d.dll | 多线程, 动态链接, 调试 | /MDd     | `_DEBUG, _MT, _DLL`           |

[^1]: 运行库的命名办法: `libcpmtd.lib`, `p` 指 C++, `mt` 指 Multi-Thread 支持多线程, `d` 支持调试. 动态链接库命名前缀为 `msvc`, 还会加上版本号, 如 Visual C++ 2005 内部版本号为 8.0, 其多线程动态链接版的 DLL 命名为 `msvcrt80.dll`.