---
tags:
---

> 还请见 [colcon coverage](../../ROS/colcon%20coverage.md)
## 插桩编译

先将源码编译为 llvm 中间表示 (IR).

`--coverage` 插桩编译, 编译后产生两种文件:
- `.gcno` 部分 cov 信息
- `.gcda` 运行后, 为每个 obj 文件单独生成. 每次执行结果都会累积到该文件, 应适时删除旧文件.

`llvm-cov gcov [bin]` 输出代码覆盖率摘要
- `-f` 显示各个函数覆盖情况
- `-b` 显示 branch 选择情况
- `-m` demangle function name. 常搭配 `-f` 一起用, mangle 意思是编译器会将函数名映射为方便机器识别(辨别)的码.
- `-u` 显示非条件性分支, 如 `return`.

## 开发日志

### 23-09-01

llvm-cov 和 gcov 都是命令行工具, 没有提供 c 接口, 只能以 shell 命令方式操作咧.

由于 gcov 工具是累加性质的, 并且只有覆盖了新分支时, cov 报告才会变化. 因此使用一个 hash 函数, 映射为 cov_info. 变化时, 才标记 interesting.