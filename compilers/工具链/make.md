makefile 基础语法:

```makefile
target:file1.c file2.c // 依赖关系

<tab>  gcc -o file file1.c file2.c // bash 命令, 注意前必须有 tab
<tab>  echo "complete" // bash 命令2, 和前命令在不同shell中并行执行.
```

一个非常离谱的例子:

```makefile
file:file.o
	gcc file.o -o file
file.o:file.s
	gcc -c file.s -o file.o
file.s:file.i
	gcc -S file.i -o file.s
file.i:file.c	
	gcc -E file.c -o file.i
```

执行 make 的逻辑结构是 **栈**. 默认执行第一条依赖关系, 即试图从 `file.o` 生成 `file`, 若依赖文件不存在, 再向下检查; 也可以直接指定生成: `make file.s`.

make 通过时间轴实现 **增量编译**. 每次运行时, 其会检查: 1 目标文件是否存在, 2 ==目标文件生成时间是否晚于其依赖文件, 以此来查看依赖文件是否有更新==. 文件时间有 ACM (access, change: 最后一次属性修改, modify: 最后一次内容修改), make 比较 mtime.

### 项目清理

```makefile
-PHONY: clean
clean:
	rm -f file.o file.s file.i
```

执行 `make clean`. `-PHONY` 指明一个伪目标, make不会去检查 clean 文件是否已存在.

### 自动变量

- `$@`: 当前规则中的目标, 如 `target: file1 file2` 中的 `target`
- `$^`: 规则中所有依赖项, 如 `file1 file2`
- `$<`: 规则中第一个依赖项, 如 `file1`
- `$?`: 规则中已更新的依赖项
- `$*`: 文件名但不包括后缀

### 惯例变量

- `CC`: c 编译器, 默认 cc
- `CXX`: c++ 编译器, 默认 g++
- `CFLAGS`: c 编译参数
- `CXXFLAGS`: c++ 编译参数

### 参数

- `make -f <file>` 指定要使用的 makefile 文件
- `make -C <directory>` 在指定目录下执行`make`
- `make -n` 显示将执行的命令, 但不真正执行它们
- `make -B` 强制执行所有目标, 而非增量编译, 或 `make --always-make`
- `make -j 4` 指定并行任务数量
- `make -s` 或 `make --silent` 只显示错误信息
- `make -k` 或 `make --keep-going` 即使中途失败, 也继续构建
- `make -p` 打印 Makefile 中的环境变量字面值 

