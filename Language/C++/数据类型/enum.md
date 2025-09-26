## enum

```c
enum Color {
	Red,
	Green,
	Blud
};

Red; // 直接暴露访问
```

C 的枚举类型, 被编译器按 `int` 处理, 不会严格检查. 因为将 `RED | GREEN` 这种位运算值当作 `Color` 类型也是允许的. 但 C++ 编译器不允许这么处理.

## enum class

```cpp
enum class Color {
	Red,
	Green,
	Blue 
};

Color::Red; // 必须通过作用域访问
```

## enum_reflection 

