共享库就是指共享对象. 共享库更新可能会破坏兼容性, 即发生了**接口改变**. C 语言
改变 ABI 的行为主要有:
- 导出函数的行为发生改变, 即调用该函数的前后结果不一致.
- 导出函数被删除.
- 导出函数的结构发生变化, 如用于数据传输的结构体定义有改变. 通常而言, 需要保证
 向结构体尾部添加成员不会导致不兼容.
- 导出函数的接口变化, 如函数返回值和参数.

C++ ABI 兼容性超级复杂, 建议使用 C 接口.

### Linux 

Linux 规定共享库的命名规则必须形如: `libname.so.x.y.z`

- `x` 指**主版本号**, 不同主版本号的库不兼容. 依赖于旧版本库的程序, 要不修改源
码并重新编译, 要不就在系统中保留旧版本的共享库.
- `y` 指**次版本号**, 表示库的增量升级. 即增加新的接口符号, 但保留原来的符号不变.
- `z` 指**发布版本号**, 对库进行错误修正或性能改进, 不添加或改变任何接口.

> GNU 显然不服 Linux, Glibc 编译器和语言库使用的命名方式是 `libc-x.y.x.so` 

### SO-NAME

由于库版本变更频繁, 导致共享库文件名也不断变更. 程序文件通过 SO-NAME 机制去追踪
实际的依赖库位置, 比如 `libfoo.so.2.6.1`, Linux 系统会创建一个 `libfoo.so.2` 的
*软链接* 指向该库, 所有可执行文件指向链接. 当库更新时, 只需要更新软链接的指向, 
即可保证所有可执行文件依赖于更新后的库.

系统安装或更新共享库时, Linux 会运行 `ldconfig` 工具, 遍历共享库目录, 然后更新
所有软链接, 使其指向最新版本 (次版本兼容) 的库. 该程序将全部 SO-NAME 缓存到 
`/etc/ld.so.cache` 中, 供动态链接器快速查找.

SO-NAME 机制解决了**向后兼容**问题, 但无法解决次版本号**向前兼容**. Linux 有另
一个**符号版本**机制用于解决前向兼容性, 即对使用的库函数*次版本号*进行标记, 
如 `VERS2.3`, 链接器会找到最高的次版本号, 应用该共享库. 该机制较复杂, 所以仅在 
Glibc 相关库中得到了应用, 没有广泛推广.

## 查找共享库

动态链接器按如下顺序查找共享库:
1. 由 `LD_LIBRARY_PATH` 变量指定的路径.
2. 在 ELF .dynamic 端中寻找 `DT_NEED`, 若为绝对路径, 则在对应目录查找.
3. 若 `DT_NEED` 中为相对路径, 则在 `/lib, /usr/lib`, 和 `/etc/ld.so.conf` 配置
文件中指定的目录相对查找. 该过程是读取 `ldconfig` 的缓存 `/etc/ld.so.cache`
4. 默认共享库目录, 先 `/usr/lib`, 再 `/lib`

## 环境变量

### LD_LIBRARY_PATH

用于临时改变程序的共享库查找路径, 而不影响其他程序. 比如使用定制版 `libc.so.6`, 
将其放入 `/home/usr` 中, 然后指定:

```sh
LD_LIBRARY_PATH=/home/user:/bin/ls ...
```

另一种办法是, **直接用动态链接器启动程序**:

```sh
/lib/ld-linux.so.2 -library-path /home/user:/bin/ls
```

### LD_PRELOAD

`LD_PRELOAD` 的优先级比 `LD_LIBRARY_PATH` 还高. 无论程序是否依赖于它们, 
`LD_PRELOAD` 指定的共享库或目标文件都会被装载. 即, 全局符号会覆盖后续加载的同名
全局符号. 

类似 `LD_PRELOAD` 作用的是配置文件 `/etc/ld.so.preload`

### LD_DEBUG

用于打印连接时调试信息, 即显示整个装载流程. 

```sh
LD_DEBUG=files ./a.out
```

`LD_DEBUG` 可以设置为:
- `bindings` 显示动态链接的符号绑定过程
- `libs` 显示共享库的查找过程
- `version` 显示符号的版本依赖关系
- `reloc` 显示重定位过程
- `symbols` 显示符号表的查找过程
- `statistics` 显示动态链接过程中的各种统计信息
- `all` 显示所有
- `files` 

## 共享库管理

### 创建

创建共享库

- `shared` 指输出结果是共享库类型的
- `fPIC` 使用地址无关代码 (Position Independent Code) 来输出文件
- `Wl` 用于将指定参数直接传递给链接器 `-soname,my_soname`
- `-soname` 指定链接器为我们的共享库创建 SO-NAME

```sh
gcc -shared -fPIC -Wl,-soname,libfoo.so.1 \
		-o libfoo.so.1.0.0 \
		libfoo1.c libfoo2.c -lbar1 -lbar2

# 等价于 
gcc -c -g -Wall -o libfoo1.o libfoo1.c
gcc -c -g -Wall -o libfoo2.o libfoo2.c
ld -shared -soname libfoo.so.1 -o libfoo.so.1.0.0 \
		libfoo1.o libfoo2.o -lbar1 -lbar2
```

应保留共享库的符号与调试信息, 同时不要使用 `gcc -fomit-frame-pointer`.

使用 `LD_LIBRARY_PATH`, 或 `ld -rpath` 指定链接时的共享库查找路径, 可用于在调试
中替换原本的标准库:

```sh
ld -rpath /home/mylib -o a.out a.o -lsomelib
```

共享模块反向引用主模块中的符号时, 可能该符号并没有放到动态符号表中, 此时反向引用
会失败. ld 提供了 `-export-dynamic` 参数用于将所有全局符号导出到动态符号表.

### 安装

如果有 root 权限, 那么将共享库复制到 `/lib, /usr/lib` 等标准目录中, 然后运行 
ldconfig. 若没有 root, 可以手动向 ldconfig 指定目录, 用于构建 SO-NAME, 然后在
编译时用 `-L` 参数指定共享库搜索目录, 或用 `-I` 指定共享库路径.

```sh
ldconfig -n /path/to/shared_lib_dir
```

### 共享库构造与析构

GCC 提供了内置的共享库构造函数和析构函数, 但并不是语言标准, 需要和 GCC 标准库
一起使用.

```c
void __attribute__((constructor)) init_func(void);
void __attribute__((destructor)) fini_func(void);
```


