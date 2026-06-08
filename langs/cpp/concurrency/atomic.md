## C++ 原子量模型

`std::atomic<T>` 提供多种内存顺序（Happens-Before 约束）：
- `memory_order_relaxed` 只保证原子性，不做任何内存顺序约束。
- `memory_order_acquire` 将当前 load 视为同步点，之后的读写不会重排到它之前。也叫*后向屏障 (backward fence)*
- `memory_order_release` 将当前 store 视为同步点，之前的读写不会重排到它之后。也叫*前向屏障 (forward fence)* 
- `memory_order_acq_rel` acquire + release. 适合三元操作 (read-modify-write)
- `memroy_order_seq_cst`: sequential consistency. 最强的 SC 顺序一致，默认行为。对比 `mo_acq_rel` 而言，`mo_seq_cst` 还会建立全局的偏序关系，不局限于单一原子变量。

大部分高级语言, 原子变量默认是强 SC 语义: 提供 atomic (不可分割) + volatile (可见性) + barrier (顺序一致性) 功能.

内存一致性相关见 [hw/computer/内存一致性](../../../hw/computer/内存一致性.md)

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

**内存栅栏**：同时影响所有原内存访问。不绑定到某一个具体原子量，因此内存栅栏**只影响本线程指令重排**，不影响多线程同步。

```cpp
data = 42;
std::atomic_thread_fence(std::memory_order_release); 
ready.store(true, std::memory_order_relaxed);
```

## 实例

#### 门卫

`release` 和 `acqurie` 一般要配合使用。

```cpp
// producer
data = 24;
ready.store(true, std::memory_order_release);

// consumer 
if (ready.load(std::memory_order_acquire)) {
	... // using data, with data = 24
}
```

* release：生产者，设置 `ready` 之前的写入都应该完成
* acquire: 消费者，看到 `ready` 后，后面的读写不能跑到这个观察动作之前。否则仍会看不到 `42`

#### 模拟锁

```cpp
// unlock 
state.store(0, std::memory_order_release);

// lock 
while(!state.compare_exchange_weak(0, 1, std::memory_order_acquire)){}
```

#### 计数器自增

```cpp
cnt.fecth_add(1, std::memory_order_relaxed);
```

无门卫作用，无需内存序约束。

