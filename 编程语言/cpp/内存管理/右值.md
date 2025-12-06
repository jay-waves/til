## 左值

*左值 (lvalue)* 指一个表示指向或引用了一个可持久存储的位置 (变量, 数组元素, 结构体成员), 可以在内存中被定位.

- 可以出现在赋值语句的左侧.
- 有明确地址, 可以被 `&` 运算符取出.
- 一般代表对象身份 (变量名)

程序中一般被称为 `lhs` (left-hand side), 相应的右值被称为 `rhs`

## 右值

*右值 (rvalue)* 指表达式不指向或不引用任何有持久存储位置的对象, 是临时并无法被识别的. 或者其值无法被修改.

- 一般出现在赋值语句的右侧.
- 没有固定内存地址, 不能被取地址.
- 通常代表对象的值, 如字面量或临时值. 因此其所有权可以合法转移.

### 右值引用

C++11 引入右值引用 `&&` 的概念, 使得资源 (如动态分配的内存) 从一个对象转移到另一个对象, 而无需复制.

```cpp
std::string str = "Hello"; // str is a lvalue
std::string another_str = std::move(str); // using std::move to move content of str to another_str 
```

`std::move` 比深度复制更加高效, 因为 another_str 直接接管了 str 的内部数据 (被 `move` 转换为右值). `=` 赋值操作默认行为是浅复制, 对象定义的拷贝赋值函数可以覆盖默认行为. `std::move` 则进一步使用移动赋值函数.

右值可以绑定到 `const T&`, 但是不能绑定到 `T&`. C++14 引入 `T&&` 用于绑定右值. 注意, 作为模板参数时, `T&&` 是万能引用, 而作为具体参数时, `string&&` 只能绑定到右值.

### `std::move`

移动语义是一种软约束, 它规定了 C++ 语言的语义而不是硬件的行为, 所以可以强制类型转换:

```cpp
String str2 = static_cast<String &&>(str1); // 强制转化为右值, 去触发移动构造函数

// 等价于 
String str2(static_cast<Stirng &&>(str1));

// STL 提供了语法糖
String str2 = std::move(str1);
```

`std::move()` 实现如下. 它其实没有改变值的左或右, 只是一个强制类型转换. 但是, **它并不会修改被移动的对象**, 两个对象会指向同一个内存空间. 要实现完整的移动语义, 需要手动在 移动构造函数 中将原对象置空. **此外, `move()` 也不会导致析构函数执行, 原对象仍按原作用域销毁.**

```cpp
template <typename T>
constexpr std::remove_reference_t<T> &&move(T &&ref) noexcept {
	return static_cast<std::remove_reference_t<T> &&>(ref);
}
```

```cpp
// 下面是错误的操作, 为什么?
MyClass&& fn() {
	MyClass obj(1);
	return std::move(obj);
}
```

### 引用折叠

在泛型编程中, 可能会出现 `T& &&` 或 `T&& &` 的推导类型, 这种类型在实际语法中是不存在的. 所以在类型推导时, 会出现*引用折叠*:
* `T& &` 折叠为 `T&`
* `T& &&` 折叠为 `T&`
* `T&& &` 折叠为 `T&`
* `T&& &&` 折叠为 `T&&`

**所以只要出现左值引用 `&`, 那么类型总会被推导为 `T&`**. 

```cpp
template <typename T>
void f(T&& x) { ... }

int i;
f(i); // 由 int& && 折叠为 int&
```

### 完美转发

**在模板参数中, `T&&` 作为万能引用, 既能绑定到右值, 也能绑定到左值.** 比 `const T&` 更灵活.

在下面的 `handle_path` 例子中, 传入右值时, `T&& path` 折叠为 `string&& path`, **函数内部的右值引用 `path`, 因为有标识符, 被 C++ 认为是一个左值**. 如果想把它仍作为右值转发给其他函数, 需要用到*完美转发 (forwarding reference)*, 即 `std::forward<T>(path)` 将其变为将亡值.

`std::forward<T>()` 可以做到:
* `T` 推导为 `U&`, `T&&` 折叠为 `U&` 时, 返回左值
* `T` 推导为 `U`, `T&&` 折叠为 `U&&` 时, 返回右值. 这个类似 `std::move()` 行为.

```cpp
template <typename T>
void wrapper(T&& x) {
	func(std::forward<T>(x));
}

void handle_path_impl(const std::string& path) {}

// 核心问题是, string&& p 这个具名变量, 并不能传递给 handle_path_impl(string&&),
// 因为它已经是一个左值了.
void handle_path_impl(std::string&& path) {}

template <typename T>
void handle_path(T&& path) {
	// 需要重载的方式
	handle_path_impl(std::forward<T>(path));
	
	// 另一种无需重载的方式
	using RawT = std::remove_reference_t<T>;
	if constexpr (std::is_lvalue_reference_v<T>) {
	
	} else {
	
	}

}

std::string p = "....";
handle_path(p); // 传左值, 走 const string& 版本

handle_path(std::string("/tmp")); // 传右值, 走 string&& 版本
handle_path(std::move(p)) ; // 传右值
```

推测和修改 `T&&` 的实际类型:

```cpp
std::is_lvalue_reference<T>::value // 需要搭配 if constexpr ()
std::is_rvalue_reference<T>::value 
std::is_reference<T>::value 
std::remove_reference<T>::type
```

## RVO

C 语言函数返回值逻辑:
1. 若寄存器 (一个或多个) 能存储下, 有限存储到寄存器中返回
2. 若多个寄存器皆存不下, 会在调用函数前先开放一片栈内存用于存储函数内部的返回值
3. 若调用方某个变量直接接受函数返回值, 就将这片内存标记给该变量
4. 若调用方只使用返回值一部分, 这片空间就会变成匿名空间.

C++ 语言函数返回值逻辑: (加上对象的析构和构造)
1. 返回值直接放在寄存器中返回
2. 返回值被转化为出参, 内部直接操作外部栈空间. 如果析构和拷贝构造函数都不是默认行为 (非平凡) 时, 为了保证对象行为的完整性, 会先构造一个局部临时变量在栈上使用, 再拷贝过来.
3. 如果用匿名空间接受函数返回值, 在处理完函数调用语句后, 匿名空间将被析构.

C++ 依据上述三种特征, 划分了三种值类型:
1. prvalue (pure rvalue, 纯右值): 寄存器保存, 对于汇编而言就是纯常数, 没有内存实体, 不能修改操作.
2. lvalue (左值): 返回值直接绑定到调用者的某个变量. 作用域结束后才会析构.
3. xvalue (expiring value, 将亡值): 返回值在某个匿名内存空间中, 完成某动作后, 就会被调用析构函数. 但它是有内存实体的.

|          | 有标识符 | 无标识符 |
| -------- | -------- | -------- |
| 不可移动 | lvalue   |          |
| 可移动   | xvalue   | prvalue         |

lvalue 和 xvalue 的区别, 导致函数的返回值不能直接取址. 比如 `&Demo()`. 

prvalue 和 xvalue 返回值都会在传递后消失, 所以 C++ 将它们统称为 *右值 (rvalue)*. 可以理解为, C++ 将寄存器抽象为无穷大, 所以就不存在 xvalue 情况, 但右值在硬件上实现并不统一.

lvalue 的对象如果非平凡, 会多一次不必要的拷贝过程. 因此 C++ 引入 *右值引用* (C++14) 和 *拷贝省略 (Copy Elision)*. NRVO 也属于一种*拷贝省略 (Copy Elision)*.

对比:

```cpp
MyObj pass() {
	MyObj obj;
	return obj; // RVO 
}

MyObj pass() {
	return MyObj(); // RVO
}

MyObj pass() {
	MyObj obj;
	return move(obj); // not RVO, only moving
}

MyObj pass(obj) {
	return obj; // not RVO
}
```

## 参考

https://www.zhihu.com/question/428340896/answer/2913419725

