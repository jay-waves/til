gcc 是驱动 ld, as, cc 等编译链工具的前端. 

编译过程: 编译 + 链接. 

将 .c 源文件预处理为 .i 文件, 再编译为 .o(.obj) 目标文件, 再链接为可执行程序 .exe.

### 选项

- `-E` 预处理后即停止. `gcc -E a.c -o a.i`
- `-S` 编译为汇编代码. `gcc -S a.i -o a.s`, Unix 系统中, `.s` 是完全的汇编语言格式, `.S` 则可以使用预处理命令 `#define` 等.
- `-c` 汇编为目标代码 `gcc -c a.s -o a.o`

- `-I` 指定头文件路径
- `-ffreestanding` 独立程序, 不链接 C runtime 和 crt.S 等. 等于 `-nostdlib -nostartfiles`
- `-g` 启用调试信息
- `-O0` 优化级别
- `-Wall` 启用大部分编译警告
- `-fno-builtin` 禁用 GCC 内置函数 
- `-verbose` 详细日志

- `-L` 指定链接时查找路径, 多路径用 `:` 隔开
- `-static` 静态链接

- `-shared` 产生共享对象, 使用[装载时重定位方法](../链接过程/动态链接.md)
- `-fPIC` [地址无关代码](../链接过程/动态链接.md)
- `-fPIE` 地址无关可执行文件

## 其他 GNU 工具

- `flex` : 词法分析器
- `bison` : 语法分析器
- `m4, cpp` 预处理器
- `lld`: list dynamic dependencies, 检测可执行程序依赖哪些库.
- `as` 汇编器, 一般由 gcc 控制.
- `ld` 链接器, 一般由 gcc 控制.
- `ar` 创建和管理静态库
- `nm` 查看符号

`objump` [可执行文件](../链接过程/Unix-ELF.md)查看器:
- `-a` 列出所有包含的目标文件
- `-C` C++ Demangle 
- `-g` 显示调试信息 
- `-d/-D` 反汇编
- `-h` 显示段表
- `-t` 显示符号表
- `-T` 显示动态链接符号表

