
## 头文件

`.h` 头文件应与其源码文件的命名保持已知, 内容包括:
- 可见函数声明, 不应包括具体实现
- 宏定义
- 复杂类型定义 `struct, enum`
- 全局变量 `extern`
- 内联函数定义 `inline`, 应包括具体实现. 编译器即实例化.

```c
// module.h
#ifndef MODULE_H // 防止重复定义
#define MODULE_H 
extern int ex_var; // 全局外部变量

void function();

inline void func() {
	...
}

typedef struct {
	int field;
} MyStruct;

#endif // MODULE_H
```

`.hpp` 头文件用于 C++, 内容除 `.h` 外还有:
- 复杂类型定义: class. 
- 模板函数定义, 应包括具体实现. 编译期即实例化.

```cpp
class MyClass {

public:
	void member_func();
	
protected:
	void protected_func();
	
private: // default member set
	int priv_var;
	void priv_func();
};

template <typename T>
class MyTemplate {
public: 
	void do_something(T value);
};
```

## 源文件

`.c` 源文件, 包括:
- 函数具体实现. **函数的定义和声明应分离, 避免重复定义风险.**
- 局部变量和函数的定义

```c
// module.c
#include "module.h"

int ex_var = 0; // 定义外部变量

static int func() { // 定义局部 (文件内) 工具函数
	...
}

void function() {
	...
}
```

`.cpp` 源文件, 还包括:
- 导出C符号函数 `extern "C"`
- 实现类的成员函数

```cpp
#include "module.hpp"

extern "C" {
	void c_func(); // c_func is defined in some c source file.
}

c_func(); // using c function

void MyClass::member_func() {
	...
}
```

## 程序入口

一个 C/C++ 程序实例只有唯一的源码级入口: `main()` 函数. 所有测试文件也应包括 `main()` 函数, 用于独立运行测试.

```
int main(int arg, char* argv[]);
```

## 程序出口

程序正常退出路径：`int main() {....; return 0;}` 

异常中止路径：
* `std::eixt()` 来自 `<cstdlib>`
* `std::abort()` 来自 `<cstdlib>`
* `std::terminate()` 异常抛出未被处理时，触发。记得 `catch (...)`
* `_exit()`  操作系统级退出进程
* 外部信号
* SEGV 

**所有异常路径，都不能保证栈正常展开，换言之 RAII 可能失效**。

## 命名空间

不要在其他 Namespace 中引入头文件，否则命名空间会嵌套。

```cpp
namespace my {
	
	#include <iostream> 
	
	using cout = std::cout; // error, no my::std::cout

}
```