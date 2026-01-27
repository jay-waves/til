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

C/C++/GO 的整数算术, 都向零取整, 直接截断小数部分.

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

注意不同大小数组是不同类型, 如 `[5]int` 和 `[10]int` 是不同的类型. 这也导致了 go 标准库函数参数通常是*切片*, 而不是数组. 

对于数组, `cap() == len()`. 

### Slice 

*切片 (Slice)* 可视为对某个数组的引用, 也可以视为动态数组. *数组 (Array)* 是值类型; *切片 / 字典* 是引用类型, 传参时默认拷贝引用而不是值. 

**声明方法:**

```go
var s []int // 切片无需长度
s := arr[start:end] // 是指 [s,e)
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

* 当不需要扩容时, 切片操作后, 新切片仍指向原数组. 如 `arr[:]`
* 当需要扩容时, 切片操作会分配新底层数组, 新切片指向新数组, 旧数据被拷贝到新数组.
* 底层数组默认分配在栈上, 但当逃逸分析成立时, 也会分配在堆上.

### Map 

字典类型属于引用类型, 并且字典的键类型只能是值类型.

```go
mmap := map[string]int{
	"one": 1,
	"two": 2,
}
```

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

### Channel 

通道类型属于引用类型, 并且其本身是并发安全的. 通道用于传递数据, 在两个 goroutine 间通信.

声明一个通道: 
- `ch := make(chan int)` 默认不带缓冲区, 必须同步收发.
- `ch := make(chan int, 100)` 设定缓冲区大小, 允许异步. 超出缓冲区会阻塞.

使用通道: 对于无缓冲通道, 接收方将阻塞同步, 直至发送动作完成.

```go
ch <- v    // 把 v 复制到通道 ch, 原 v 仍可用.
v := <-ch  // 从 ch 移动出数据, ch 中数据被销毁.
close(ch) //关闭通道
```

缓冲区通道: The $kth$ receive on a channel with capacity $C$ is synchronized before the completion of the $(k+C)th$ send from that channel completes.

```go
var limit = make(chn int, 3)
func main() {
	for _,w := range work {
		go func(w func()){
			limit <- 1
			w()
			<-limit
		}(w)
	}
	select{}
}
```

单向通道: 主要用于参数约束

```go
type Notifier interface {
	SendInt(ch chan<- int)
}
```

### 指针

获取地址 `&var`. 不可取地址类型:
- 常量
- 临时结果
- ...

引用内容 `*var`

空指针为 `nil`

### 类型判断

类型断言表达式: `x.(T)`
* `x` 是接口类型的值, 比如 `interface{}(var1)`. `interface{}` 代表空定义的接口.
* `value, ok = x.(T)`

类型别名:

```go
type MyString = string // 别名

type MyString string // 类型重定义, 两者是不同类型
```

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

### function

函数: go 中函数为一等公民 (First Class), 即可以作为参数和变量进行传递.

```go
func MyFunc( [parameter list] ) [return_types | (return_type1, return_type2, ...)] {
   函数体
}
```



### select 

哪个通道中有可用元素, 就走哪个分支. 
* `case` 条件表达式至少包含一个 `<-`
* 存在 `default` 分支时, `select` 语句不会阻塞.
* 所有 `case` 条件表达式都会被求值一次, 因此不要在 `case` 条件表达式中使用有副作用的函数.
* 当多个 `case` 均满足条件时, Go 使用伪随机算法挑选一个执行.

```go
select {
case <- ch1:
	...
case <- ch2:
	...
case _, ok := <- ch3:
	if !ok {
		...
		break //仅跳出 select
	}
default:
	...
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

## 测试

* TestXXX 
* BenchmarkXXX
* ExampleXXX