默认用于 Windows 上的汇编语法, 也是 Intel 操作手册上的标准语法. 特点是目的操作数在前, 源操作数在后.


寄存器不需要前缀:
```asm
mov eax, ebx
```

寄存器命名

```
eax

mov eax, 10

mov eax, [ebx]
```

## Hello World

`helloworld.asm`, 包含两部分:
- `.data` 段用于定义常数和初始化变量
- `.text` 段用于定义指令

在 `.text` 段中, 用 `global` 符指明程序入口, 这里为 `_start`.

```x86asm
section .data
	; 定义常量 msg, 类型为 db (define byte) 字符串
	msg db `Hello, World!\n`

section .text
	; code here
	global _start

_start:
	; 使用 sys_write 系统调用, id 为 1.
	;     size_t sys_write (uint fd, const char* buf, size_t count)
	;     将 id 挪入 rax
	;     将参数 fd=1 (stdout) 挪到 rdi. rdi 固定接受第一个参数. 
	;     将参数 buf=msg 挪到 rsi 中, 
	;     将 count=14 挪到 rdx 中. 注意 msg 最后需要添加 `\0`, 所以长度为 14
	mov rax, 1
	mov rdi, 1
	mov rsi, msg
	mov rdx, 14
	syscall

	; 使用 sys_exit 系统调用, id 为 1, 退出程序. 
	;      void exit(int status)
	;      将 status=0 挪到 rdi 中
	mov rax, 60
	mov rdi, 0
	syscall
```