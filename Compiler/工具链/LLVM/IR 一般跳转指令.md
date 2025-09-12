## 控制流

### icmp

```llvm
%result = icmp uge i32 %a, %b
```

`icmp` 接受的第一个参数 `uge` 称为**比较策略**, 有很多种:
- `eq`, `ne` 相等或不相等
- 无符号比较: `ugt` 大于, `uge` 大于等于, `ult` 小于, `ule` 小于等于
- 有符号比较: `sgt`, `sge`, `slt`, `sle`

### BasicBlock

LLVM 的函数由一系列基本块 (Basic Block) 组成, 形成一个 CFG (Control Flow Graph). 每个基本块由一个 label 标识, 以终止指令结束 (通常是跳转指令).

### for 语句

```c
for (int i=0; i<4; i++){
	// for body 
}
// next
```

`i` 是可变变量, 必须声明在内存中. Llvm 不推荐这种方式, 因为内存比寄存器慢.

```llvm
entry:
	%i = alloca i32                   ; int i
	store i32 0, ptr %i               ; i=0
	br label %check
check:
	%i_val = load i32, ptr %i
	%result = icmp slt i32 %i_val, 4              ; test if i <4
	br i1 %result, label %for_body, label %next   ; 条件跳转
for_body:
	%1 = add i32 %i_val, 1            ; %1 <- i+1, 匿名寄存器 %1.
	store i32 %1, ptr %i              ; i <- %1, 从寄存器取数
	br label %check                   ; 无条件跳转
next:
	ret i32 0
```

#### phi 

当多个控制流路径 (分支, 循环) 汇合到一个点时, 由于 SSA 策略, 需要一种机制来选择不同路径上的变量值. `phi` 指令可以根据控制流来向, 选择对应的去向.

用 phi 方式写上述 for 循环:

```llvm
entry:
	br label %check 
check:
	%i_val = phi i32 [0, %entry], [%i_next, %for_body]
	%result = icmp slt i32 %i_val, 4
	br i1 %result, label %for_body, label %next 

for_body:
	%i_next = add i32 %i_val, 1
	br label %check 

next:
	ret i32 0
```

### switch 语句

```c
// in c
int x;
switch (x) {
	case 0:
		// case a
		break;
	case 1:
		// case b
		break;
	default:
		// case c
		break;
}
// next
```

```llvm
switch i32 %x, label %C [
	i32 0, label %A
	i32 1, label %B
]
A: 
	; case a
	br label %end
B:
	; case b
	br label %end
C:
	; default case, must provided
	br label %end
end:
	; next
```

### select 语句

`select` 语句和 `phi` 类似, 但是其仅适用于**二选一**的情况.

```c
// in c
void foo(int x){
	int y;
	if (x > 0){
		y = 1;
	} else {
		y = 2;
	}
}
```

使用 `select` 类似 python/rust 中的单行表达式: `let y = if x>0 {1} else {2}`

```llvm
; use select
define void @foo(i32 %x){
	%1 = icmp sgt i32 %x, 0
	%y = select i1 %1, i32 1, i32 2
	ret void
}
```
