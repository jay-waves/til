## concept 

传统模板是对一组类型的抽象描述, 它通常对所定义的类型有隐式要求. 比如 `swap` 要求类型 `T` 必须支持拷贝构造和赋值, 否则会报错. 但是传统模板的报错非常抽象, 编译器输出非常冗长并且难以理解. `concepts` 比 C++11 引入的 `type_traits` 更易用.

```cpp
template<typename T>
void swap(T& a, T& b) {
	T tmp = a;
	a = b;
	b = tmp;
}
```

C++20 引入 *概念 (Concept)* 来显式对传入模板类型 `T` 的性质进行约束. 比如显式定义 `Swappable`, 编译器可以报告更高层次的错误, 代码调试更加清晰.

```cpp
template<typename T>
concept Swappable = requires(T a, T b) {
	T{a}; // 拷贝构造
	a = b; // 拷贝赋值
};

template<Swappable T>
void swap(T& a, T&b) {
	T tmp = a;
	a = b;
	b = tmp;
}
```

### 类型约束概念

```cpp
#include <concepts>

template<std::integral T>
... // T 必须是整数

template<std::floating_point T>
... // T 必须是浮点数

template<typename T, typename U>
requires std::same_as<T, U>
... // T 和 U 类型必须相同

template<typename T>
requires std::convertible_to<T, double>
... // 要求 T 可以安全(隐式)转换为类型 double
```

### 能力约束概念

基础概念:

| concept                 | 作用                                                                  |
| ----------------------- | --------------------------------------------------------------------- |
| `equality_comparable`   | 支持相等比较 `==, !=`                                                         |
| `totally_ordered`   | 支持全序比较 `<, >, <=, >=, ==, !=`                                                         |
| `swappable`             | 可交换                                                                |
| `default_initializable` | 可默认构造, 如 `T t;`                                                          |
| `copy_constructibale`   | 可拷贝构造                                                            |
| `move_constructibale`   | 可移动构造                                                            |
| `copyable`              | `movable<T> && copy_constructible<T>`. in c++, copyable implies movable                                |
| `movable`               | `swappable<T> && move_constructible<T>`                               | 
| `regular`               | `copyable<T> && default_initializable<T> && equality_comparable<T>` |

```cpp

teplate <class T, class U>
concept equality_comparable = 
	requires(
		const remove_reference_t<T>& t,
		const remove_reference_t<U>& u
	) {
		{ t == u } -> boolean; // boolean 要求结果用于布尔上下文, 如 if()
		{ t != u } -> boolean;
		{ u == t } -> boolean;
		{ u != t } -> boolean;
	};

// 应用此概念
template<std::equality_comparable T>
bool is_equal(T a, T b) {
	return a == b;
} 

template<std::totally_ordered T>
T max(T a, T b) {
	return (a > b) ? a : b;
} 

teramplate <class I>
concept weakly_incrementable = 
	semiregular<I> && requires(I i) {
		typename iter_difference_t<I>;
		requires signed_integral<iter_difference_t<I>>;
		{ ++i } -> same_as<I&>;
		i++;
	};

std::invocable<F, Args...>  // 要求 F 是可调用对象, 用 Args... 调用

std::predicate<F, Args...> // 要求 F 返回 bool

std::iterator_traits 

std::derived_from<Derived, Base> // 派生关系

std::swappable_with<T, U>  // swap(T, U) 合法

std::assignable_from<T, U> // T = U 合法 

std::range<R> // 支持 begin(r), end(r)
```

### 配合 `if constexpr`

编译时常量表达式判断, 可以让编译器激进地优化分支.

```cpp
template <typename T>
void process(T value){
	if constexpr (std::is_integral_v<T>){ // c++17
		// int
	} else if constexpr (std::is_floating_point_v<T>){
		// float
	} else {
		static_assert(std::is_same_v<T, std::string>, "unsupported type");
		// String
	}
} // only one branch will appear in compiled code

std::enable_if_t<std::is_integral_v<T>, T> processIntegral(T value) {
	
} // on or off.
```

cpp 最后会和 python 越长越像...
```cpp
template <typename... Args>
void print(Args&&... args){
	((std::cout << args << " "), ...);
}

// in c
#define print(format, ...) printf(format, __VA_ARGS__)
```