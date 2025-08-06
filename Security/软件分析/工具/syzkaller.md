1. 所有被测试系统跑在虚拟机中, 系统用户态中运行 `syz-executor`, 通过 `syscalls` 向内核输入数据. 
2. 用户通过 `ssh` 远程连接虚拟机中系统.
3. 内核通过 `kcov` 收集覆盖率信息.

## Syscalls

syzkaller 通过 `sys/$OS/*.txt` 中定义的模板来描述 syscalls. 在运行时, 通过模板来生成实际的 syscalls, 并编译后执行.

## Coverage 

syzkaller 使用 [Sanitizer Coverage](Sanitizer%20Coverage.md) (用于其他 OS) 和 `KCOV` (用于 Kernel) 来收集覆盖率. 