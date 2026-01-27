C++23 支持更多用于替代 *C 预处理器*的编译时特性, 本篇介绍这些特性[^1].

[^1]: https://learnmoderncpp.com/2023/12/29/replacing-the-preprocessor-in-modern-c/#more-2458

## 替代 `#define`

不需要借助宏来定义 常数, 内联函数, 泛型.

```cpp
#define e 2.718281828  // c preprocessor style, compile-time constant
constexpr double e = 2.71828 // c++11, compile-time constant, with specified-type
const double e = exp(1.0) // compile-time or *run-time* constant

#define times_two(n) (2*(n))
inline int times_two(int n) {return 2*n;}

#define square(t,n) t square(t n) { return n*n; } // c
template <typename T>
T square(T n) { return n*n; } // c++
// 非泛型:
double square(double n) { return n*n; }
long long square(long long n) { return n*n; }
```


## 别名

```cpp
using size_t = unsigned int;
```

## 用 `static_assert()` 替代 `#error`

`#error` 多用于编译预处理阶段检查错误.
```cpp
static_assert(sizeof(int)==4, "unsupported int width")
#if SIZEOF_INT != 4 // c
	#error "unsupported int width"
#endif
```

同理, 应该用异常处理 exception 来代替 assert, 因为 `assert()` 通常是宏实现的, 没有上下文. 详见 [error-handling](error-handling.md)

## 模块, 而不是 `#include`

使用 `import`, 比 `#include<...>` 导入标准库更快.

```cpp
import std; // c++23
```

