---
code: [<future>]
---

`<future>` 是比直接控制 线程/锁 更高层的调用, 专注于异步执行任务.

## future 

一个任务想要向另一个任务传递值, 它会做出一个 `promise`. 在未来某时刻, 该值会生效, 在对应的 `future` 内部. `promise` 可以传递值, 也可以传递一个[异常](../error-handling.md)

```cpp
void producer(std::promise<int> p) {
	j
	try {
		...
		p.set_value(res);
	} catch (...) {
		p.set_exception(some_exception());
	}
}

void consumer(std::future<int> f) {
	int v = f.get(); // blocks until producer set value
}

{
	std::promise<int> p;
	std::future<int> f = p.get_future();
	
	auto f1 = std::async(std::launch::async, producer, std::move(p));
	auto f2 = std::async(std::launch::async, consumer, std::move(f));
	
	// wait for both
	f1.get();
	f2.get();
}
```

用 `future` 另一个优势是自动 RAII 管理线程，不需要手动。默认会隐式 `join()`；强制退出时，也会保证 terminated. 

`std::future` 在C++11 和 C++14 引入，不过个人认为 C++20 `std::jthread` 更易用，因为支持 `std::stop_token` 取消机制。详见 [thread](thread.md#jthread)