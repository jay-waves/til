## 纯函数

*纯函数 (Pure Function)*:
* 函数的结果只受参数影响, 没有对环境依赖. 
* 函数产生的唯一影响, 就是其返回的结果. (无副作用, Side-Effect Free)

由此产生的特点:
* 管道化: 函数易于互相组合
* 不变性: 不鼓励传入引用, 因为其可导致外部状态改变. 
* 函数头等公民: 函数像变量一样, 被赋值 / 传递 / 操作.


## 闭包

Closures, 使函数能够 "记住" 其定义时的环境. 

## 柯里化和偏应用

Partial Application & Curring. 将一系列多参数转换为单参数函数.

## 递归与尾递归

**尾调用**指函数 `f()` 最后调用 `g()`, 此时 `f()` 不必等待 `g()` 返回才释放资源.


```go
func f(x){
	return g(x)
}
```

```go
// 不是尾调用, 但编译器可以优化为尾调用
func f(x) {
	y := g(x)
	return y
}

// 这也不是尾调用
func f(x){
	return g(x)+1
}
```

*尾调用优化 (Tail Call Opt)* 指函数 `f()` 尾调用 `g()` 后立即释放自己的调用栈帧 (call frame), 取代为 `g()` 的调用栈. 可以节省栈内存.

*尾递归*指函数 `f()` 尾调用 `f()` 自己. 使用尾调用优化, 可以避免递归导致的*栈溢出*错误. 实质是: 递归总能写成等价循环.

```go
// 不适用尾调用的递归
func factorial(n){
	if n==1 {return 1}
	return n*factorial(n-1)
}
factorial(5)

// 使用尾调用
func factorial(n, total){
	if n==1 {return total}
	return factorail(n-1, n*total)
}
factorial(5, 1)
```

使用尾调用的递归 `factorail` 要传入 `total`, 比较不自然, 可以改写:

```go
func tail_factorial(n, total){
	if n==1 {return total}
	return factorial(n-1, n*total)
}
func factorial(n){
	return tail_factorial(n,1)
}
```

使用*柯里化 (curring)*, 将多参数函数转换为单参数形式:
```python
def currying(fn, n):
	def inner(m):
		return fn(m,n) # 延迟调用
	return inner

def tail_factorial(n, total):
	if n == 1: return total
	return tail_factorial(n - 1, n * total)

factorial = currying(tail_factorial, 1)

factorial(5)
```

采用默认值
```python
def factorial(n, total=1):
	if n==1:
		return total
	return factorial(n-1, n*total)
factorial(5)
```

## 惰性求值

*惰性求值 (Lazy Evaluation)* 指表达式只有在需要时才会被计算, 而不是立即执行, 以此提高整体效率.

## 模式匹配

通过*模式匹配 (Pattern Matching)* 来简化条件逻辑. 将数据集 + 数据操作 + 返回值都紧凑到一起, 减少控制逻辑带来的干扰.

## 高阶函数

使用函数作为参数的函数. 

### map

### reduce 

### filter 

### Y 组合子

## 修饰器模式



## 参考

- [尾调用优化 - 阮一峰的网络日志](https://www.ruanyifeng.com/blog/2015/04/tail-call.html)
