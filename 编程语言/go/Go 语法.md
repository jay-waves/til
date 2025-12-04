Go 语言是编译型面向过程语言, 语法类似C. 为*并发*设计专用语法, 引入 `GC` 和内存安全机制.

注释: `//` 或 `/**/`

## 数据结构

1. 布尔值: `true` `false`. *go 语言中只有布尔值可以作为判断条件*
2. 数字类型: 支持复数, 位运算使用补码
3. 字符串: utf-8 标识的 Unicode 文本, 由字节流组成.
4. 派生类型

### 数字


| Length | Signed | Unsigned | Blob | float | complex |
| ------ | ------ | -------- | ---- | ----- | ------- |
| 8b     | int8   | uint8    | byte |       |         |
| 16b    | int16  | uint16   |      |       |         |
| 32b    | int32  | uint32   | rune | float32      |        |
| 64b    | int64  | uint64   |      | float64      | complex64        |
| arch   | uint   | int      |      |       |         |


### 结构体

```go
type StructType struct {
   member definition
   member definition
   ...
   member definition
}
```

声明结构体时, GC 会进行*逃逸分析*, 判断: 若变量可能在作用域外访问, 在堆上分配.
```go
// 返回结构体值，此后所有赋值都是值拷贝
var person Person = Person{}
// 返回指针， 而不是值拷贝
var person *Person = &Person{}
```

Go 语言中, 变量和指针都使用 `.` 来访问结构体成员.

### Array

声明: `var arrayName [size]dataType`

初始化: 
```go
var numbers = [5]int{1, 2, 3, 4, 5}
// same as
numbers := [...]int{1, 2, 3, 4, 5}
```

注意不同大小数组是不同类型, 如 `[5]int` 和 `[10]int` 是不同的类型. 这也导致了 go 标准库函数参数通常是*切片*, 而不是数组. 任意大小数组转化为切片: `array[:]`

### Slice 

*切片 (Slice)* 可视为对某个数组的引用, 也可以视为动态数组. *数组 (Array)* 是值类型, 默认在栈上分配; *切片 / 字典* 是引用类型, 传参时默认拷贝引用而不是值.

**声明方法:**

```go
var s []int // 切片无需长度
s := arr[start:end]
s := make([]int, len, capacity)
s := []array{} // s:= []*Node{&my_node}
```

切片底层是一个结构体:
```go
type slice struct { 
	array unsafe.Pointer // 指向底层数组的指针 
	len int // 切片的长度 
	cap int // 切片的容量 
	}
```
*当使用一个数组的切片时, 如 `array[:]`, 其底层数据仍指向 array*


### Map 

### String

`string` 不可变. 性能差, 用于只读文本.

`[]byte` 可变, 存储二进制数据. 在 `capacity` 内操作切片无需内存分配.

组合两个切片:
```go
slice1 := []byte("hello, ")
slice2 := []byte("world!")
result := append(slice1, slice2...)
// ...是展开符号, 将slice2元素全部拆开

var buffer bytes.Buffer
buffer.Write(slice1)
buffer.Write(slice2)
result := buffer.Bytes()
```



### 指针

获取地址 `&var`. 不可取地址类型:
- 常量
- 临时结果
- ...

引用内容 `*var`

空指针为 `nil`

## 变量初始化

未指定初始值时, 变量初始化为*零值*:

| 类型               | 值             |
| ------------------ | -------------- |
| 数值               | `0`            |
| 布尔               | `false`        |
| 字符串             | `""`           |
| 结构体             | `StructType{}` |
| `*int, []int`      | `nil`          |
| `map[string] int`  | `nil`          |
| `chan int`         | `nil`          |
| `func(string) int` | `nil`          |
| `error`            | `nil`               |


为了避免*未初始化访问*错误, Go 规定, 除了全局变量, 其他变量声明后必须使用. 否则编译器将报错.

## 控制流语句

匹配:

```go
switch var1 {
    case val1:
        ...
        fallthrough //fallthrough 强制执行后面一个case
    case val2:
        ...
    default:
        ...
}
```

条件:

```go
if a > b {
	...
}
```

循环: for 有三种写法 

```go
for init; condition; post {} // C for
for condition {} // C while
for {} // loop
```

迭代元素, 如字符串/数组/切片:

```go
for key, value := range oldMap {
    newMap[key] = value
}
```

函数: go 中函数为一等公民 (First Class), 即可以作为参数和变量进行传递.

```go
func MyFunc( [parameter list] ) [return_types | (return_type1, return_type2, ...)] {
   函数体
}
```


## 错误处理

Go 语言鼓励显式处理错误, 以提高代码可靠性. 

go 的错误处理基于一个内置接口 `error`. 注意接口 `error` 本身的类型是*接口*, 空值为 `nil`

```go
type error interface {
    Error() string
}
```

go 的标准函数通常会伴随返回一个错误, 通过与 `nil` 比较来检查是否出现了错误:
```go
result, err := SomeFunction()
if err != nil {
    // 处理错误
} else {
    // 处理结果
}
```

使用 `erorrs.New()` 方法可返回一个错误消息.
```go
func Sqrt(f float64) (float64, error) {
    if f < 0 {
        return 0, errors.New("math: square root of negative number")
    }
    // 实现
}
```

自定义错误类型, 可以在接口中附加更多调试信息:
```go
type MyError struct {
    Message string
}

func (e *MyError) Error() string {
    return e.Message
}
```

打印错误信息:
```go
err := fmt.Errorf("An error occurred: %w", originalErr)
```

注意, Go 并不强制处理 `err`, 可以用 `_` 匿名变量来忽略掉. GO 的 `return result, err` 实际有四种组合状态, 而 Rust `Result<T, E>` 只有 `Ok(T)` 和 `Err(E)` 两种, 并且 Rust 强制处理错误.

