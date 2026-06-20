
## 类继承

```cpp
class Animal{
	...
}

class Cost{
	...
}

class Cat: public Animal, public Cost{
	...
}
```

C++11 `final` 关键字, 禁止其他类继承该类. 即声明该类为叶子类.

```cpp
class Base final {
	...
};
```

## 友类

访问控制权限:

| 访问   | public | protected | private |
| ------ | ------ | --------- | ------- |
| 同一类 | Y      | Y         | Y       |
| 子类   | Y      | Y         | N       |
| 外部类 | Y      | N         | N       |
| 友类   | Y      | Y         | Y        |

```cpp
class A{
public:
	friend class B;
private:
	int m_;
}

class B{
public:
	void get_m(A& a) {cout << a.m_}
}
```
- 友元关系不是相互的, 即A声明B为友元, 那么A能访问B的所有成员, 而B不能访问A.
- 友元关系无法继承

## 多态

C++ 多态指: 父类使用 `vitual` 声明一虚函数, 而不给出具体实现 (通过 `fn() = 0`, 不同子类对同一父类方法进行不同实现. 

C++ 虚函数本质是使编译器不静态链接导到函数, 而是运行时通过*虚函数表 (vtbl)* 来动态确定重定位位置. 虚函数实现方式详见 [runtime-poly](runtime-poly.md)

```cpp
class Animal{
public:
	virtual void f(); // 虚函数, 可重载
	virtual void g() = 0; // 纯虚函数, 即父类不会实际定义, 子类必须定义.
	// 声明纯虚函数的类称为抽象类, 不能被实例化.
	
	virtual void h() override; // 用于子类重载父类函数.
	virtual void k() override final; // final 指后续派生类不能再重定义该函数.
}
```

即使是同一类, 借助 [c++ 函数签名包含参数的特性](../name-mangling.md), 类也可以有同名方法. 比如, 多种(参数不同的)初始化方法:

```cpp
class Person{
	public:
		Person() = delete; // Person p1{}, p2(), p3
		Person(const Person&) = default;  // Person p4{p1}, p5(p2)
		Person& operator= (const Person&) = delete; // 符号重载, Person p6; p6=p1
		Person(Person&&) = delete; // Person p7{std::move(p2)};
		Person& operator= (Person&&) = delete; // Pserson p8; p8=std::move(p3)
		~Person() = delete; // 析构函数
}
```

### 隐藏 vs. 多态

```cpp
class Base {
	virtual void xxx();
	void yyy();
};

class Yield : Base {
	void xxx() override; // 多态, 动态绑定
	void yyy(); // 只是同名隐藏, 不能 override
};

Yield obj;
Base* ptr = &obj;

ptr->xxx(); // --> Yield.xxx
ptr->yyy(); // --> Base.yyy
```

### 虚继承

为了解决菱形继承问题而引入的*虚继承*. 普通继承中, 子类 (B, C) 会持有一份父类 A 对象, 这导致孙子类 (D) 会有两份不同的 A. 导致 *二义性问题*.
```cpp
struct A { int a; };
struct B : A { int b; };
struct C : A { int c; };
struct D : B, C { int d; };
```

使用虚继承, D 持有的 B/C 中, 都指向同一个 A. 注意, B/C 独自实例化时, 行为不会改变, 仍持有独立的 A. 

D 持有 B 和 C, 两者内部的 A 变为虚基指针 `(virtual A)*`, 指向同一个派生 A. 在 MSVC 上称之为 `vbptr`. 

```cpp
struct A;
struct B : virtual A;
struct C : virtual A;
struct D : B, C;
```

当存在多个虚基类时, 只存储一个 `vbptr` 不够. 此时 MSVC 将 `vbptr` 指向一个 `vbtable`, 在表中存储每个虚基类的派生实例的偏移. 在 GCC/Clang 上, `vbtable` 实际上存储在 `vtable` 中, 不单独使用 `vbptr`, 而复用 `vptr`.

```cpp
struct X: virtual Y, virtual Z;
```

