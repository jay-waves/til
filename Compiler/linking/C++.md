### 符号改编

多模块目标文件的符号名可能重复, C 语言提供一些作用域限定符 (`extern, static`) 来限制符号污染, C++ 则提供了命名空间 (namespace) 和符号改编 (name mangling) 机制. 

C++ 扩展了 C 语言的**函数签名 (function signature)**, 以准确识别函数符号, 其内容包括: 函数名, 参数类型, 命名空间, 所属类. 函数签名不同, 函数即不同, 符号不会冲突, 被称为 C++ **函数重载**.

```cpp
int func(int);
float func(int);
class C {
	int func(int);
	class C2 {
		int func(int);
	};
};

namespace N {

int bar;
int func(int);
class C {
	int func(int);
};

} // namespace N
```

在底层编译时, C++ 会对函数签名进行修饰, 形成独特的符号名, 称为符号改编. *注意, 符号改编不考虑返回值, 也不考虑变量类型*. 考虑仅有返回值不同的两个 `func()`, 由于 C 具有隐式类型转换, 对 `func()` 的调用不能仅凭返回类型来确定.

| 函数签名[^1]           | g++ 符号改编后    |
| ---------------------- | ----------------- |
| `int func(int)`        | `_Z4funci`        |
| `float funct(int)`     | `_Z4funcf`        |
| `int C::func(int)`     | `_Z41C4funcEi`    |
| `int C::C2::func(int)` | `_ZN1C2C24funcEi` |
| `int N::func(int)`     | `_ZN1N4funcEi`    |
| `int N::C::func(int)`  | `_ZN1N1C4funcEi`  |
| `bar`                  | `_ZN1N3barE`                  |

[^1]: 例子来源于 *程序员的自我修养--链接, 装载与库*, 俞甲子等, P88.

g++[^2] 符号改编基本规则如下.
- 普通符号 `_Z` 开头; 嵌套符号路径以 `_ZN` 开头, `E` 结尾.
- 后续路径名称为*作用域名称长度+作用域名称*
- 符号路径后是参数列表, 如 `int -> i`, `string -> Ss`, `long -> l`

[^2]: 不同编译器的符号改编规则可能不同, 可使用 `c++filt` 等工具反解析符号.

### extern "C"

C++提供了 `extern "C"` 关键词来导出 C 符号, 而不使用 C++ 的符号改编方式. 常用于和 C 语言兼容.

```c
extern "C" {
	int func(int);
	int var;
}

extern "C" int func(int);
```

当 C 和 C++ 公用一套头文件时, 为了避免导出符号不一致, 常用办法为:

```c
// g++ will define __cplusplus for c++ programs.
#ifdef __cplusplus 
extern "C" {
#endif

void *memset (void *, int, size_t);

#ifdef __cplusplus
}
#endif
```
