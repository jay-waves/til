## 类型推导

### auto

### decltype

比 typeof() 更强的类型推导. C++11 引入.

```cpp
int a = t;
decltype(a + 2) b = 7; // 编译期推导 表达式结果类型

template<typename T1, typename T2>
auto multiply(T1 a, T2 b) -> decltype(a * b) {
	return a * b;
}

// 直接推导函数调用的返回类型
decltype(add(1, 2)) result = add(3 + 2);
```

C++11 标准规定如下:

```cpp
int x = 10;
decltype(x) a = 20; // 未加括号变量名, 推导变量的声明类型 int
decltype((x)) b = x; // (x) 视为左值表达式, 推导为 int&
decltype(x + 1) c = 11; // x + 1 是右值表达式, 推导为 int
```

与 `auto` 相比, `decltype` 会默认保留引用和 `const` 修饰符.

```cpp
int x = 10;
const int& ref = x;

auto a = ref;        // a --> int
decltype(ref) b = x; // b --> const int&
```

## 类型擦除

### any

### variant
