C++ 协程是无栈的. 协程的调用栈 `A()->B()->C()`, 在协程切换时无法保存, 必须自然退出. 因此无栈协程从挂起中恢复时, 不能恢复挂起时的调用栈, 只能依赖编译器构造的状态机. 

对于 C++ 而言, 协程的本地数据被编译器保存在*协程帧 (coroutine frame)*, 包括所有 局部变量 / PC / Promise / `std::coroutine_handle<>`, 类似一个快照. 对于每个 `co_await, co_yield, co_return`, 都会变成状态机的一个分支, 该状态机在每次恢复时跳到上次被挂起的位置, 并恢复 coroutine frame 中的数据.

要注意, 将局部变量保存在协程帧的过程, 是编译器自动完成的. 所有在 `co_await` 之前没有结束生命周期的对象, 都会被放在协程帧上. 这可能导致内存管理问题.

### awaitable

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

因此, 要自定义可挂起类型 `awaitable`, 需要至少三个接口
* `await_ready()`
* `await_suspend()`
* `await_resume()`

比如: 

```cpp
struct std::suspend_always {
	bool await_ready() const noexcept 
	{
		return false;
	}
	
	void await_suspend(coroutine_handle<>) const noexcept {}
	void await_resume() const noexcept {}
};

struct std::suspend_never;
```

awaitable 是一个临时对象, 被隐式保存在协程帧 (coroutine frame) 中. 而 `promise_type` 并不主动持有 awaitable 对象. 协程帧的大致结构:

```
coroutine frame {
	promise_type 
	local variables 
	awaitable (expr)
	suspension state
}
```


### resumable

* resumable: 是协程实际的生命周期管理者, 用于在最外层 (外部) 控制协程调度.
* awaitable: 仅定义协程挂起/恢复的行为.
* coroutine_handle: 指向 coroutine frame. 每个协程帧包括一个 promise_type, 以及多个 awaitable, 以及其他控制语句如 `done(), resume()`.
* promise_type: 代表返回值. 同时也定义协程的一些行为.
* callable: 协程体, 定义整个协程状态机的入口.

```cpp
class T_resumable {
public:
	struct promise_type {...};
	
	using coro_handle = std::coroutine_handle<promise_type>;
	
	explicit xxx_resumable(coro_handle handle) : handle_(handle) {}
	...
	
	bool resume() {
		if (!handle_.done()) 
			handle_.resume();
		return !handle_.done();
	}
	
	uint64_t get() {
		return handle_.promise().value_;
	}

private:
	coro_handle handle_; // 核心是, 持有一个 coroutine_handle 对象
};


// promise_type 用于定义一些协程执行的关键步骤
// 并保存协程的最终结果, 或需要跨协程上下文调度的长期状态.
// 但是, promise_type/resumable 应该和 外部调度器, awaitable 充分解耦.
struct promise_type {
	T value_;
	using coro_handle = std::coroutine_handle<promise_type>;
	
	// callable 返回值
	auto get_return_object() {
		return T_resumable{coro_handle::from_promise(*this)};
	}
	
	// 创建时是否理解挂起
	constexpr auto initial_suspend(); 
	// 结束时是否挂起
	constexpr auto final_suspend() {
		return suspend_always();
	}
	
	// 正常执行协程体时, 其内部可能有 co_await, co_yield 的挂起行为. 
	// 执行结束后, 根据返回值类型, 执行 promise 中的函数.
	
	// 正常退出
	void return_void();
	// 异常处理
	void unhandled_exception() {
		std::termintae();
	}
	// co_yield 时, 实际调用了 co_await promise.yield_value
	auto yield_value(T value) {
		// 这里是一个示例
		value_ = value;
		return suspend_always();
	}
	
};
```

### callable

这仅仅是一个返回 `resumable` 对象的函数, 告知编译器这里是协程状态机入口. 编译器会寻找 `T_resumable::coroutine_handle::get_return_object()`, 从而获取该协程的实际 `promise_type`, 接着用它来构建协程帧. 每调用一次 `foo()`, 就创建一个协程.
```cpp
T_resumable foo() {
	co_await std::suspend_alwayes{};
	co_await std::suspedn_never{};  // 自由地使用各类 awaitable
	co_await xxx_IO_awatable(fd);
	// ....
	co_return;
}
```

## 实例

这里使用 `cppcoro` 库.

### 协程的组合方式

串行 (seq):

```cpp
auto a = co_await A();
auto b = co_await B();
co_return combine(a, b);
```

并发 (parallel):

```cpp
auto ta = A();
auto tb = B();
co_await when_all(ta, tb);
```

管道 (pipeline):

```cpp
auto gen = generator();
auto parser = parser(gen);
auto writer = writer(parser);
```

扇出扇入 (fan-out/fan-in):

```cpp
auto results = when_all(A(), B(), C());
```

取消或重试 (timout, cancellation):

```cpp
auto result = retry(A(), timeout_token);
```

### 异步 IO

协程有三种使用形式:

```cpp
task<> server() {
	char data[1024];
	while (true) {
		size_t n = co_await socket.async_read_some(data);
		co_await async_write(socket, ...);
	}
}

```

```cpp
task<int> foo() {
	int v= co_await bar();
	co_return v + 1;
}
```

### 生成器

```cpp
generator<unsigned int> iota(unsigned int n = 0) {
	while (true)
		co_yield n++;
}
```

### 惰性求值

```cpp
lazy<int> f() {
	co_return 7;
}
```