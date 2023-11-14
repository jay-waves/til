## bitmap

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