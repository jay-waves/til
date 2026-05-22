*条件变量 (condition variable)* 使线程等待某一条件 (事件, 由其他线程引发).

```cpp
#include <condition_variable>
...

queue<Message> mqueue;
condition_variable mcond;
mutex mmutex;

void consumer() {
	while(true) {
		unique_lock<mutex> lck {mmutex};
		while(mcond.wait(lck));

		auto m = mqueue.front();
		mqueue.pop();
		lck.unlock();
		// ... process m ...
	}
}

void producer() {
	while(true) {
		Message m;
		// ... fill the message ...
		unique_lock<mutex> lck {mmutex};
		mqueue.push(m);
		mcond.notify_one(); 
	} // release lock implicitly
}
```

c++ 线程在条件变量 `wait` 后不会轮询等待，而是进入睡眠态，放弃 CPU 占用。当显式调用 `notify()` 时，才会唤醒等待线程。但是，线程可能因为其他原因唤醒，如内核调度、信号等，C++ 标准要求 `wait` 至少要放在 `while(predicate)` 循环中。

当使用 `cv.wait(lck, pred)` 时，`notify()` 只是将线程唤醒，如果不满足 `pred` 线程仍会继续睡眠。这个过程（唤醒又睡眠）可能有性能损耗。

```cpp
c.wait(lock, pred);

// 等价于：先判断条件。如果条件不符，再次 wait
while (!pred()) {
	cv.wait(lock); /*
		1. 释放 lock 持有的 mutex, 注意前提条件是持有 mutex
		2. 线程立即进入睡眠，触发 OS 调度
		3. 适时被 notify 唤醒
		4. 阻塞尝试  lock(mutex)
	*/
}
```

如果使用 `notify_all()` ，要注意所有线程被唤醒后都会竞争同一个 `mutex` ，这导致大部分线程阻塞在 `lock(mutex)` 上。此时 C++ 不保证他们会睡眠，libc 会在短时间内自旋尝试获取锁，失败后进入 slow path 进入操作系统 futex 调度。