
| `std::funciton`       | `virtual`      | `override` |
| --------------------- | -------------- | ---------- |
| `[]()->type{}` lambda func | `operator()()` |            |


在运行时选择具体调用哪个函数[^1]. 最传统的方法是使用 `if-else, switch` 分支语句:

```cpp
if (...)
	return f(...);
else
	return g(...);
```

[^1]: https://learnmoderncpp.com/2023/11/22/selecting-functions-at-runtime/#more-2406

### 函数指针

注意用 `(*)` 免于受运算符优先级影响.
```cpp
int(*pf1)(double, double) = &floor_divide; // c-style funciton pointer

using PFType = int(*)(double, double);     // modern c++ using-declaration
PFType pf2 = &floor_divide;

(*pf1)(7.0, 3.3)
(*pf2)(7.0, 3.3)
```

### 虚函数

ABC (Abstract Base Class) 纯虚类型定义虚函数, 支持运行时多态. 详见 [面向对象](面向对象/面向对象.md#多态)

vtbl...

### 函数体

[函数对象](类型系统/STL/辅助函数.md)指重载了 `()` 操作符的类. 下面的代码基于 cpp 函数重载, 用虚参数 (但类型确定) 决定触发的具体方法, 这种技术称为标签派发( tag-dispatch ).

```cpp
class DivideCeiling {};
class DivideFloor {};

class DivideToInt {
	double divisor_, dividend_;
public:
	DivideToInt(double divisor, double dividend)
		: divisor_( divisor ), dividend_( dividend ) {}
	int operator() (const DivideCeiling&) {
		return ceil(divisor_ / dividend_);
	int operator() (const DivideFloor&) {
		return floor(divisor_ / dividend_);
	}
};

int main() {
	DivideToInt division(7.0, 3.3);
	division(DivideCeiling{})
}
```

### 匿名函数

C++11

```cpp
[捕获列表] (参数列表) -> 返回类型 {
	函数体
}
```

- 捕获列表: 定义该Lambda表达式可以从本地作用域捕获哪些变量.
- 参数列表: Lambda表达式接受的参数
- 返回类型

```cpp
#include <iostream>
#include <vector>
#include <algorithm>

int main() {
    std::vector<int> numbers = {1, 2, 3, 4, 5};

    // 打印每个元素
    std::for_each(numbers.begin(), numbers.end(), [](int number) {
        std::cout << number << std::endl;
    });

    // 计算所有元素的和
    int sum = 0;
    std::for_each(numbers.begin(), numbers.end(), [&sum](int number) {
        sum += number;
    });
    std::cout << "Sum: " << sum << std::endl;

    return 0;
}
```

#### 将匿名函数用于延迟求值

lazy evaluation 概念源于函数式编程, 含义为当结果被明确要求时, 求值行为才进行.

```cpp
int x;
y = x * 2 + 1; // just a number
auto y = [&x]{ return x * 2 + 1; }; // a caller
```

通过传入引用的方式, y 的结果实时依赖于 x.


#### 匿名函数捕获

lambda 表达式定义局部函数, **支持闭包**, 便于自定义函数与 STL 交互 (如 `sort(), search()` 传入的 `cmp` ). 但实际上 C/C++ 并不支持定义**局部函数**, 即在函数内部定义子函数, lambda 表达式技术上是通过定义匿名体来实现的:

```cpp
auto add = [](int a, int b) { return a + b; };

// convert to:
class __lambda_unique_class {
public:
    int operator()(int a, int b) const { return a + b; }
} add;
```

> c++ 真是和 python 狼狈为奸, 快长成亲兄弟了. 
> 
> lambda 实际是先在闭包函数里定义结构体, 把闭包里局部变量的引用定义为结构体成员; 然后结构体上定义方法, 伪装成类; 然后再把类的 `()` 操作符重载, 让它支持调用; 然后再伪装成匿名函数. 这事整的....

这导致匿名函数的真实函数签名非常复杂 (取决于闭包函数, 以及捕获的局部变量), 所以存储匿名函数的变量类型一般使用 `auto`.  

C++ 也提供了 `std::function` 来简化对匿名函数的签名定义.

```cpp
#include <functional>
#include <initializer_list>
enum class DivisionPolicy { Ceiling, Floor };

int main(){
	std::function<int(double, double)> df;
	for (auto p: {DivisionPolicy::Ceiling, DivisionPolicy::Floor}):
		switch (p) {
			case DivisionPolicy::Ceiling:
				df = [](double a, double b)-> int { return ceil(a/b); };
				break;
			case DivisionPolicy::Floor:
				df = [](double a, double b)-> int { return floor(a/b); };
				break;
		}
		df(7.0, 3.3);
}
```

### 字典查询

借用 STL 的 `unordered_map` 数据结构, 实现类似 python 的效果.

```cpp
#include <funcitonal>
#include <unordered_map>
#include <initializer_list>
#include <math.h>

using namespace std;

unordered_map<string, function<int(double, double)>> DivisionPolicy {
	{"ceiling", [](double a, double b)-> int { return ceil(a/b); }},
	{"floor", [](double a, double b)-> int { return floor(a/b); }},
};

DivisionPolicy["ceiling"](7.0, 3.3);
```