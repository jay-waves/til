CAS, *Compare-And-Swap*. 将给定值 `expected` 与内存值 `*address` 比较，如果它们是同一个值，就用新值 `desired` 替换 `*address` 中的值，整个过程通过原子指令完成。

```c
atomic {
	bool CAS (address, expected, desired) {
		if (*address == *expected) {
			*address = desired;
			return true;
		} else {
			*expected = *address;
			return false;
		}
	}
}
```

CAS 相当于**无锁的进行一次性操作保护**, 即确保只有一个线程能成功执行某些操作. 但是它比如下的这种写法更快:

```cpp
std::mutex m;
atomic<bool> running = true;

void join(std::thread& t) {
	std::lock_guard<std::mutex> lk(m);
	if (running) {
		if (t.joinable()) t.join();
		running = false;
	}
}

// CAS 方法
void join(std::thread& t) {
	bool expected = true;
	if (running.compare_exchange_strong(expected, false))
		if (t.joinable()) t.join();
}

// atomic 方法
std::atomic_flag gate = ATOMIC_FLAG_INIT; 

void join(std::thread& t) {
	// 只有第一次返回 false
    if (!gate.test_and_set(std::memory_order_acq_rel)) { 
        if (t.joinable()) t.join();
    }
}
```

用 CAS 实现简单自旋锁:

```cpp
void lock() {
	while (flag.exchange(true, std::memory_order_acquire))
		; // busy wait
}

void unlock() {
	flag.store(false, std::memory_order_release);
}

```

用 cas 实现 `add1`. 能保证，**读-修改-写入**过程不会被打断.

```go
func add1(val *int32) (new int32) {
	for {
		v := *val 
		if cas(val, v, v + 1) {
			return v + 1;
		}
	}
}
```