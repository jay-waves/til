
## 函数

函数和全局变量都会被记录进符号表.

```llvm
define dso_local void @foo() #0 {
  ret void
}

attributes #0 = { noinline nounwind optnone uwtable "frame-pointer"="all" "min-legal-vector-width"="0" "no-trapping-math"="true" "stack-protector-buffer-size"="8" "target-cpu"="x86-64" "target-features"="+cx8,+fxsr,+mmx,+sse,+sse2,+x87" "tune-cpu"="generic" }

; #0 称为属性组, 可以代指定义的一长串属性. 属性直接写在函数后也是可行的, 比如:
define dso_local void @foo() noinline nounwind {
	ret void
}
```

```c
// in c
void foo() {}
```

需要使用其他模块的函数时, 需要在本模块先使用 `declare` 声明该函数:

```llvm
declare i32 @printf(i8*, ...) #1
```

LLVM IR 调用函数的方式和高级语言类似:

```llvm
define void @bar() {
	%1 = call i32 @foo(i32 1)
}
```

### 内置函数 (Intrinsics)

LLVM IR 提供了一些内置的基础函数实现, 详见 `include/llvm/IR/Intrinsics*.td`.

内存操作:
- `llvm.memcpy.`
- `llvm.memmove`
- `llvm.memset`
- `llvm.lifetime.start/end` : 标记内存对象声明周期
- `llvm.invariant.start/end` : 标记内存区域只读
- `llvm.prefetch`
- `llvm.atomic.*`

位操作:
- `llvm.ctpop`
- `llvm.ctlz`
- `llvm.cttz`
- `llvm.fshl`
