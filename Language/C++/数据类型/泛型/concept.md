## concept 

传统模板是对一组类型的抽象描述, 它通常对所定义的类型有隐式要求. 比如 `swap` 要求类型 `T` 必须支持拷贝构造和赋值, 否则会报错. 但是传统模板的报错非常抽象, 编译器输出非常冗长并且难以理解.

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

C++20 定义的一些常用 Concept:

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

template<std::equality_comparable T>
bool is_equal(T a, T b) {
	return a == b;
} // 要求 T 支持 == 和 != 比较语义

template<std::totally_ordered T>
T max(T a, T b) {
	return (a > b) ? a : b;
} // 要求 T 支持全序比较: <, >, <=, >=, ==, !=

template<std::movable T>
void move_xxx(T&& value) {
	T local = std::move(value)
} // 要求 T 是可移动的, 并且是可交换的

template<std::default_constructible T>
T create() {
	return T{};
}  // 要求 T 有默认构造函数, 可以无参数初始化

std::regular // 正则的, 即: 支持默认构造, 拷贝构造, 赋值和相等比较.

std::invocable<F, Args...>  // 要求 F 是可调用对象, 用 Args... 调用

std::predicate<F, Args...> // 要求 F 返回 bool

std::iterator_traits 

std::derived_from<Derived, Base> // 派生关系

std::swappable_with<T, U>  // swap(T, U) 合法

std::assignable_from<T, U> // T = U 合法 

std::regular<T>  // 满足基本语义: copyable, equality, default_constructible 

std::totally_ordered<T>  // 支持 < > <= >=

std::default_initializable<T> // T t 合法

```

`<type_traits>` 是 C++11 引入的, 基于模板偏特化 (SFINAE), `<concepts>` 是其升级版, 更易用.

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