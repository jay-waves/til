## C++ 原子量模型

`std::atomic<T>` 提供多种内存顺序（Happens-Before 约束）：
- `memory_order_relaxed` 只保证原子性，不做任何内存顺序约束。
- `memory_order_acquire` 将当前 load 视为同步点，之后的读写不会重排到它之前。也叫*后向屏障 (backward fence)*
- `memory_order_release` 将当前 store 视为同步点，之前的读写不会重排到它之后。也叫*前向屏障 (forward fence)* 
- `memory_order_acq_rel` acquire + release. 适合三元操作 (read-modify-write)
- `memroy_order_seq_cst`: sequential consistency. 最强的 SC 顺序一致，默认行为。

大部分高级语言, 原子变量默认是强 SC 语义: 提供 atomic (不可分割) + volatile (可见性) + barrier (顺序一致性) 功能.

内存一致性相关见 [arch/co/内存一致性](../../../hw/computer/内存一致性.md)

## 接口

```cpp
namespace std {

enum class memory_order;

template<class T> struct atomic {
	bool is_lock_free()); // ???
	
	T load(memory_order); /* memory_order = memory_order::seq_cst by default */
	void store(T, memory_order);
	
	T exchange(T, memory_order);  // swap
	bool compare_exchange_weak(T&, T, memory_order, memory_order); // cas 
	bool compare_exchange_strong(T&, T, memory_order, memory_order); // cas 
	// weak cas would require a loop and a strong on would not.
	
	T fetch_add(T, memory_order);
	T fetch_sub(T, memory_order);
	T fetch_and(T, memory_order);
	T fetch_or(T, memory_order);
	T fetch_xor(T, memory_order);
	T fetch_max(T, memory_order);
	T fecth_min(T, memory_order);
	
	// memory_order_seq_cst:
	T operator++(int);
	T operator--(int);
	T operator+=(T);
	T operator-=(T);
	T operator&=(T);
	T operator|=(T);
	T operator^=(T);
};

struct atomic_flag {
	bool test(memory_order);
	bool test_and_set(memory_order);
	void clear(memory_order);
};

}
```

C++20 引入了轻量的停等机制，类似 [<condition_variable>](condition_variable.md)

```cpp
struct atomic {
	void wait(T expected, memory_order); 
	
	void notify_one();
	void notify_all();
}
```

## 实例

#### 门卫

```cpp
// producer
data = 24;
ready.store(true, std::memory_order_release);

// consumer 
if (ready.load(std::memory_order_acquire)) {
	... // using data, with data = 24
}
```

#### 模拟锁

```cpp
// unlock 
state.store(0, std::memory_order_release);

// lock 
while(!state.compare_exchange_weak(0, 1, std::memory_order_acquire)){}
```

#### 原子读

```cpp
cnt.fecth_add(1, std::memory_order_relaxed);
```

无门卫作用，无需内存序约束。