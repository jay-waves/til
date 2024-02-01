## Hello World

```llvm
; main.ll
define i32 @main() {
	return i32 0
}
```

`clang main.ll -o main`

### Data Layout

```llvm
target datalayout = "e-m:e-p270:32:32-p271:32:32-p272:64:64-i64:64-f80:128-n8:16:32:64-S128"
```

这一长串文字就定义了目标的数据布局。具体而言：

- `e`: 小端序
- `m:e`: 符号表中使用ELF格式的命名修饰
- `p270:32:32-p271:32:32-p272:64:64`: 与地址空间有关
- `i64:64`: 将`i64`类型的变量采用64位的ABI对齐
- `f80:128`: 将`long double`类型的变量采用128位的ABI对齐
- `n8:16:32:64`: 目标CPU的原生整型包含8位、16位、32位和64位
- `S128`: 栈以128位自然对齐

> 参考 [Data Layout](https://llvm.org/docs/LangRef.html#data-layout)

```
+------------------------------+
|          stack_data          |
|         heap_pointer         |  <------------- stack
+------------------------------+
|                              |
|                              |  <------------- available memory space
|                              |
+------------------------------+
| data pointed by heap_pointer |  <------------- heap
+------------------------------|
|          global_data         |  <------------- .data section
+------------------------------+
```