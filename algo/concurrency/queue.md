线程安全队列，普通锁实现。

```cpp
template <class T>
class Queue {
public:
	Queue(const Queue&) = delete;
	Queue& operator=(const Queue&) = delete;
	
	void push(T val);
	
	auto try_pop() -> optional<T>;
	
	auto wait_and_pop() -> T;
	
	bool empty() const;
	
	size_t size() const;
	
private:
	mutable mutex mtx_;
	condition_variable cv_;
	queue<T> q_;
};
```

`push` 加锁，写入后通知潜在的消费者。

```cpp 
void push(T val) {
	{
		lock_guard lock(mtx_);
		q_.push(move(val));
	}
	cv_.notify_one();
}
```

`pop` 分为阻塞版本和非阻塞版本

```cpp
auto try_pop() -> optional<T> {
	lock_guard lock(mtx_);
	if (q_.empty())
		return nullopt;
		
	T val = move(q_.front());
	q_.pop();
	return val;
}

// 阻塞版本
auto wait_and_pop() -> T {
	unique_lock lock(mtx_);
	cv_.wait(
		lock, [this] {return !q_.empty(); }
	);
	
	T val = move(q_.front());
	q_.pop();
	return val;
}
```

