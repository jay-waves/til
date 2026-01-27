---
code: <future>
---

`<future>` 是比直接控制 线程/锁 更高层的调用, 专注于异步执行任务.

## future 

一个任务想要向另一个任务传递值, 它会做出一个 `promise`. 在未来某时刻, 该值会生效, 在对应的 `future` 内部. `promise` 可以传递值, 也可以传递一个[异常](../dev-tools/error-handling.md)

```cpp
void f(promise<X>& px) {
	// ...
	try {
		X res;
		// ... compute a value for res ...
		px.set_value(res);
	} catch (...) { // could not compute res 
		px.set_exception(current_exception()); // 向 future 线程传递一个 exception 
	}
}

void g(future<X>& fx) { // another task, get the result from fx
	try {
		X v = fx.get(); // if necessary, wait for the value to get computed 
		// ...
	} catch (...) {
		// couldnot compute v
	}
}
```

## packaged task 

`packaged_task` 

`async()` 发起线程, 以类似函数调用的方式.