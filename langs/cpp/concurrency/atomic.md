## C++ 原子变量模型

`std::atomic<T>` 提供多种内存顺序（Happens-Before 约束）：
- `memory_order_relaxed` 只保证原子性，不做任何内存顺序约束。
- `memory_order_acquire` 将当前 load 视为同步点，之后的读写不会重排到它之前。也叫*后向屏障 (backward fence)*
- `memory_order_release` 将当前 store 视为同步点，之前的读写不会重排到它之后。也叫*前向屏障 (forward fence)* 
- `memory_order_acq_rel` acquire + release. 适合三元操作 (read-modify-write)
- `memroy_order_seq_cst`: sequential consistency. 最强的 SC 顺序一致，默认行为。

大部分高级语言, 原子变量仅提供 SC 语义: 提供 atomic (不可分割) + volatile (可见性) + barrier (顺序一致性) 功能.

内存一致性相关见 [arch/co/内存一致性](../../../arch/计算机组成/内存一致性.md)

### 门卫

```cpp
// producer
data = 24;
ready.store(true, std::memory_order_release);

// consumer 
if (ready.load(std::memory_order_acquire)) {
	... // using data, with data = 24
}
```

### 模拟锁

```cpp
// unlock 
state.store(0, std::memory_order_release);

// lock 
while(!state.compare_exchange_weak(0, 1, std::memory_order_acquire)){}
```

### 原子读

```cpp
cnt.fecth_add(1, std::memory_order_relaxed);
```

无门卫作用，无需内存序约束。