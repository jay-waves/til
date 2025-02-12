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

### link

### dumpbin

类似 objdump, 查看可执行文件.

```sh
dumpbin /ALL hello.obj > hello.txt

dumpbin /SUMMARY hello.obj
```

### 修改 VS 语言

Visual Studio Installer 里 "修改", "语言", 即可修改语言. 