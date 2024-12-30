## bitmap

AFL 中, 统计edge的覆盖率信息. 考虑如下基本块路径:
```
A
|\
| \
|  B
| /
|/
C
```

如果A可以直接到C, 也可以经过B再到C. 那么仅统计ABC**基本块覆盖率**时, 即使ABC都被覆盖, 也不能说明AC路径被执行. 这样的控制流被称为"critical".

SanCov 和 AFL 提供了 edge 插桩功能: 将一个虚拟块 D 插到 critical edge 上.
```
A
|\
| \
D  B
| /
|/
C
```

wiki 上说, branch coverage 是 edge coverage 的一个子集. branch cov 统计 `if, while` 等分支语句的出现次数, edge cov 指两个基本块间的可能路径数, 两者统计方式不同. AFL 使用 edge cov, llvm bb cov 使用 edge cov.

**AFL 为每个 bb (基本块) 用编译时随机数编号, 然后用哈希函数 `hash=(bb1>>1)^bb2` 的结果表示两个基本块之间的 edge, 记录在 bitmap 中.**

## Fork Server

```c
#include ...

void forkserver_loop() {
	while (1) {
		wait_for_pipe(); // 等待 fuzzer 发送指令
		pid_t pid = fork();

		if (pid > 0) {
			// 父进程, 将子进程 pid 发送给 fuzzer
			write_pipe(pid);
		} else {
			// 子进程, 跳出函数执行原有代码
			return;
		}
	}
}

int main() {
	forkserver_loop(); // 循环等待
	/* 程序原有代码 */
}
```