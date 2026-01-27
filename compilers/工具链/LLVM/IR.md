## Hello World

```llvm
; main.ll
define i32 @main() {
	return i32 0
}
```

`clang main.ll -o main`

### LLVM IR Layout

- Module
	- Target Information
	- Global Sysmbols
		- **Global Variable**
		- **Funciton Declaration**
		- **Funciton Definition**
	- Other Stuff
- Function 
	- **Argument**
	- Entry Basic Block 
	- **Basic Block**
- Basic Block 
	- Label 
	- **Phi Instruciton**
	- **Instruction**
	- Terminator Instruction

### 编译目标平台信息

![|500](../../../attach/Pasted%20image%2020240229161828.avif)

目标的数据布局. 具体而言:

- `e`: 小端序
- `m:e`: 符号表中使用 ELF 格式的命名修饰
- `p270:32:32-p271:32:32-p272:64:64`: 与地址空间有关
- `i64:64`: 将 `i64` 类型的变量采用 64 位的 ABI 对齐
- `f80:128`: 将 `long double` 类型的变量采用 128 位的 ABI 对齐
- `n8:16:32:64`: 目标 CPU 的原生整型包含 8位、16位、32位 和 64位
- `S128`: 栈以 128位 自然对齐

`target triple`: [目标平台](../../../os/syscalls/libc.md), 如 `aarc64-apple-darwin`, `x86_64-pc-windows-msvc`


> 参考 [LLVM/LangRef/DataLayout](https://llvm.org/docs/LangRef.html#data-layout)

## 符号

以 C 语言为例, 其主要符号类型有:
```c
int a;                 // 定义在当前文件的全局变量, 别的文件也可以使用.
extern int b;         // 定义在其他文件的全局变量, 当前文件需要使用该符号.
static int c;          // 定义在当前文件的全局变量, 别的文件不可使用该符号.
void d(void);          // 定义在别的文件的函数, 当前文件需要使用该符号.
void e(void) {}        // 定义在当前文件中的函数, 别的文件可以使用该符号.
static void f(void) {} // 定义在当前文件的函数, 别的文件不可以使用该符号.
```

翻译为对应的 IR: (IR 中全局变量和函数以 `@` 开头). 

```llvm
@a = dso_local global i32 0, align 4
; dso_local 明确告知 linker 不许抢占
@b = external global i32, align 4
@c = internal global i32 0, align 4

declare void @d()

define dso_local void @e() {
	ret void
}

define internal void @f() {
	ret void
}
```

## 寄存器与栈

LLVM IR 是 SSA (Static Single Assignment) 形式, 这使得数据使用起来像"寄存器". 在编译期后端的"寄存器分配过程"中, LLVM 才决定哪些数据放在寄存器, 哪些数据放在栈中. 

在需要**直接操作地址或需要可变变量**时, 也可如下声明内存变量, 此时返回值是指针. 内存变量 / 全局变量 / 其他指针类型 都需要通过 `load/store` 来访问. 

```llvm
@global_var = global i32 0
%local_var = alloca i32

store i32 1, ptr @global_var

%1 = load i32, ptr @global_var
%2 = add i32 1, %1
```

### SSA

**SSA 中每个寄存器变量在其声明周期内只被赋值一次.** SSA 有利于编译器分析.

```llvm
; 这是非法 IR 程序
%1 = add i32 1, 2
%1 = add i32 3, 4
```

当值可能来在不同指令路径时, 需要使用 `phi` 指令显式合并: 比如这里的 `y` 可能来自 `y1, y2`.

```llvm
define void @foo(i32 %x){
entry:
	%1 = icmp sgt i32 %x, 0
	br i1 %1, label %btrue, label %bfalse

btrue:
	%y1 = add i32 0, 1
	br label %end

bfalse:
	%y2 = add i32 0, 2
	br label %end

end:
	%y = phi i32 [ %y1, %btrue ], [ %y2, %bfalse ]
	ret void
}
```

## 类型系统

官方文档中指出 LLVM 类型有:
- Void Type
- Function Type
- First Class Types
	- Single Value Types
		- Integer Type
		- Floating-Point Types
		- X86_mmx Type
		- Pointer Type
		- Vector Type
	- Label Type
	- Token Type
	- Metadata Type
	- Aggregate Types
		- Array Type
		- Structure Type
		- Opaque Structure Types

IR 的所有变量皆是强类型的, 并且没有隐式类型转换. 

### 数据类型

- 空类型 `void`
- 整型 `i1`, `i8`, `i16`, `i32`, `i64` 等. `i1` 用于标识 true/false.
- 浮点型 `float`, `double`

LLVM IR 默认数字有符号, **以补码形式**存储, 但是提供了针对无符号数的操作符 (如 `udiv`, `add`). 是否要将一个变量视为无符号数, 要看具体参与的指令.

LLVM IR 提供了明确的**类型转换指令**:
1. 长整型变为短整型, 直接裁切掉高位: `trunc i32 257 to i8`
2. 短整型变为长整型, 高位零扩展: `zext i8 -1 to i32`
3. 短整型变为长整型, 符号扩展: `sext i8 -1 to i32`
4. 浮点数和整型间转换, 有: `fptoui .. to`, `uitofp .. to` `sitofp .. to`

### 指针类型

类型 `ptr` 类似 C 语言的 `void *`, 是不包含类型信息的. LLVM 提供指针类型和整型相互转换的指令, 方便灵活操作指针.

```llvm
%x = alloca i32
%addr_x = ptrtoint ptr %x to i64
%also_x = inttoptr i64 %addr_x to ptr
```

> LLVM 11 之前, 指针是有类型的. 新版本, 指针变得"不透明".

### 聚合类型

LLVM IR 也有类似 C 语言的结构体和数组.

```llvm
; 数组
@global_arr = global [4 x i32] [i32 0, i32 1, i32 2, i32 3]

; 字符串初始化, 转义字符 \00 (固定字长为两位)
@global_str = global [12 x i8] c"Hello world\00"
```

```llvm
; 结构体
@global_struc = global {i32, i8} {i32 1, i8 0}
```


LLVM IR 要求: 访问聚合类型时, 必须指明基地址偏移 (用 `getelementptr` 计算偏移). 

```llvm
; MyT { x, y }  <--- t_ptr
%MyT type {i32, i32}

%my_y_ptr = getelementptr %MyT, ptr %t_ptr, i64 2, i32 1
%my_y_val = load i32, ptr %my_y_ptr
```

级联访问:

```llvm
%MyT = type { i32, [5 x i32] }
%t_ptr = alloca [ 4 x %MyT ]

; 取 val[2].y[3]
%1 = getelementptr %MyT, ptr %t_ptr, i64 2, i32 1, i64 3

; llvm12 后, 引入弱类型指针
%arr_ptr = alloca [3 x i32 ]
%2 = getelementptr [3 x i32], [3 x i32]* %arr_ptr, i64 0, i64 0
```

寄存器中的聚合类型, 使用 `extractvalue, insertvalue` 访问:

```llvm
%1 = load %MyT, ptr %t_ptr 

%2 = extractvalue %MyT %1, 1
%3 = insertvalue %MyT %1, i32 233, 1
```

