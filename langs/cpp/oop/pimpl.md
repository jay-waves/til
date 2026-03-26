
### PIMPL

Pointer-to-implementatoin. 在类的实现之外，再封装一层类作为接口，从而完全隐藏实现细节。当类所在的头文件被下游大量引用时，任何对头文件的更改都会导致下游重新编译，而 PIMPL 可以避免这个问题，允许该类的细节被便捷地修改。

PIMPL 的另一个作用：该类需要引用其他库文件，如 `windows.h` ，但不想将其暴露给下游。对于跨平台保持接口一致性比较有用。

```cpp 
// .h
class myclass_impl; // 仅前置声明

class myclass {
public:
	void func();

private:
	// class myclass_impl; // 也可以直接写在类内
	unique_ptr<myclass_impl> pimpl;
};


// .cc 

class myclass_impl {
public:
	void func();

private:
	int a; 
	int b;
};

myclass::myclass() {
	pimpl = make_unique<myclass_impl>();
}

myclass::func1() {
	pimpl->func();
}


// 使用 unique_ptr 时，必须让析构函数看到完整的 myclass_impl 定义
// 否则，仅前置声明会报错
myclass::~myclass() = default;
```