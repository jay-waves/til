C++ 协程是无栈的, 因此推荐结构化使用.

协程有三种形式:

```cpp
task<> server() {
	char data[1024];
	while (true) {
		size_t n = co_await socket.async_read_some(data);
		co_await async_write(socket, ...);
	}
}

generator<unsigned int> iota(unsigned int n = 0) {
	while (true)
		co_yield n++;
}

lazy<int> f() {
	co_return 7;
}
```

```cpp
task<int> foo() {
	int v= co_await bar();
	co_return v + 1;
}
```

编译器生成一个*协程帧 (coroutine frame)*, 存储所有 局部变量 / PC / Promise / `std::coroutine_handle<>`, 类似一个快照. 对于每个 `co_await, co_yield, co_return`, 都会变成状态机的一个分支, 该状态机在每次恢复时跳到上次被挂起的位置. 

对于所有的 `co_await`, 编译器将遵照 awaiter 协议生成:
```cpp
auto awaiter = expr.operator co_await();
if (!awaiter.await_ready()) { 
	awaiter.await_suspend(handle);
	... // suspend current coroutine...
	return; 
}
result = awaiter.await_resume();
```

`co_await` 是一个挂起点, awaiter (类似调度器) 将协程帧注册到异步 网络IO / 定时器 / CPU任务队列 (或优先队列) 中, 等待任务事件触发.





### generator 

