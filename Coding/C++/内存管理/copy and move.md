
| `&` | `&&` |
| -------- | --- |
| `std:move`         |     |

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

`std::move()` 完成了类型转换过程, 并确保右值对象被使用完后, 析构函数被调用. 除此之外, `move` 没有额外功能, 它只是告诉编译器可以从当前被转换对象中"掠夺"资源, 并不归还. "掠夺" 过程, 即移动构造和赋值函数, 也需要手动实现. 由于该对象的析构函数被调用, 必须确保此析构不会导致错误.

```cpp
// 下面是错误的操作, 为什么?
MyClass&& fn() {
	MyClass obj(1);
	return std::move(obj);
}
```

## 拷贝语义

C++ 默认的拷贝是*浅拷贝*. 如果需要*深拷贝*, 需要手动实现*拷贝构造函数 (copy constructor)* 和*拷贝赋值函数 (copy assignment)*.

```cpp
class Vector {
private:
	double *elem_;
	int size_;
public:
	Vector(int s);
	~Vector() { delete[] elem_; }

	// copy constructor
	Vector(const Vector& a) : elem_{new double[a.size_]}, size_{a.size_} {
		for (int i = 0; i != size_; ++i)
			elem_[i] = a.elem_[i];
	}
	// copy asignment 
	Vector& operator=(const Vector& a) {
		double *p = new double[a.size_];
		for (int i = 0; i != a.size_, ++i)
			p[i] = a.elem_[i];
		delete[] elem_;
		elem_ = p;
		size_ = a.size_;
		return *this;
	}

	double& operator[](int i);
	const double& operator[](int i) const;
};
```

## 移动语义

```cpp
class Vector {
	// ...
	// move constructor
	Vector(Vector&& a) : elem_{a.elem_}, size_{a.size_} {
		a.elem_ = nullptr;
		a.size_ = 0;
	}
	// move assignment
	Vector& operator=(Vector&& a);

	// ...
};
```

## RVO

C 语言函数返回值逻辑:
1. 若寄存器 (一个或多个) 能存储下, 有限存储到寄存器中返回
2. 若多个寄存器皆存不下, 会在调用函数前先开放一片栈内存用于存储函数内部的返回值
3. 若调用方某个变量直接接受函数返回值, 就将这片内存标记给该变量
4. 若调用方只使用返回值一部分, 这片空间就会变成匿名空间.

C++ 语言函数返回值逻辑: 加上对象的析构和构造
1. 返回值直接放在寄存器中返回
2. 返回值被转化为出参, 内部直接操作外部栈空间. 如果析构和拷贝构造函数都不是默认行为 (非平凡) 时, 为了保证对象行为的完整性, 会先构造一个局部临时变量在栈上使用, 再拷贝过来.
3. 如果用匿名空间接受函数返回值, 在处理完函数调用语句后, 匿名空间将被析构.

C++ 依据上述三种特征, 划分了三种值类型:
1. prvalue (pure rvalue, 纯右值): 寄存器保存, 对于汇编而言就是纯常数, 没有内存实体, 不能修改操作.
2. lvalue (左值): 返回值直接绑定到调用者的某个变量. 作用域结束后才会析构.
3. xvalue (expiring value, 将亡值): 返回值在某个匿名内存空间中, 完成某动作后, 就会被调用析构函数. 但它是有内存实体的.

第 2 和第 3 种类型的区别, 导致函数的返回值不能直接取址. 比如 `&Demo()` 通常会报错, 因为返回的是 左值 还是 将亡值 需要编译器根据上下文判断. 

第 1 和第 3 种类型返回值都会在传递后消失, 所以 C++ 将它们统称为 *右值 (rvalue)*. 可以理解为, C++ 将寄存器抽象为无穷大, 所以就不存在第 3 种情况, 但右值在硬件上实现并不是唯一的, 

第 2 种类型 (左值) 的对象如果非平凡, 会多一次不必要的拷贝过程. 因此 C++ 引入 *右值引用* (C++14) 和 *赋值省略 (Copy Elision)*.

```cpp
void Demo() {
	Test &&t = Demo1();
}
```

在语义上, 右值引用标识了**函数返回值直接作为实参**, 它说明在这个函数调用上下文中, 函数返回值将作为左值 (情况 2), 而不是将亡值 (情况 3). 后来, 右值引用又被用来标识 *移动语义*, 有移动语义的构造函数就是 移动构造函数, 是一种浅复制而不是深拷贝. 

如果我们想用 左值 执行/触发 移动语义, 而不是用 右值引用 呢? 因为移动语义是一种软约束, 它规定了 C++ 语言的语义而不是硬件的行为, 所以可以强制类型转换:

```cpp
String str2 = static_cast<String &&>(str1); // 强制转化为右值, 去触发移动构造函数

// 等价于 
String str2(static_cast<Stirng &&>(str1));

// STL 提供了语法糖
String str2 = std::move(str1);
```

`std::move()` 实现如下. 它其实没有改变值的左或右, 只是一个强制类型转换. 但是, **它并不会修改被移动的对象**, 两个对象会指向同一个内存空间. 要实现完整的移动语义, 需要手动在 移动构造函数 中将原对象置空.

```cpp
template <typename T>
constexpr std::remove_reference_t<T> &&move(T &&ref) noexcept {
	return static_cast<std::remove_reference_t<T> &&>(ref);
}
```


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

