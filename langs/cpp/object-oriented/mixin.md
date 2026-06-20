
## MixIn

基类通过 Abstract 接口, 要求子类实现某种协议; 通过 MixIn 接口, 给子类直接扩展功能. 
在 C++ 中, 可以通过继承模板类来静态实现. 

```cpp
template <typename T> 
struct Mixin : T {
	
};
```

## CRTP 

Curiously Recurring Template Pattern. 和多态不同, CRTP 能直接在基类中访问子类 `this`.

```cpp

template <typename Derived>
struct Base {
	void foo() {
		// ...
		*static_cast<Derived*>(this)
	}

};

struct Foo : Base<Foo>
{
	...
};
```

CRTP 也可以写出类似 *装饰器* 的效果:

```cpp
template<class Inner>
struct Logging {
    Inner inner;

    void run() {
        log();
        inner.run();
    }

    void log() {}
};

struct Task {
    void run() {}
};

auto x = Logging<Task>{Task{}};
```