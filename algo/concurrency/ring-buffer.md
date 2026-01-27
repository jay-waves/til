---
revised: 26-01-27
---


## SPSC (1producer1consumer)

ring-buffer 的单线程版本可以参考 [kfifo](../linked-list/queue.md)。

为什么 Producer-Consumer 问题不适宜使用队列？
* 显式维护 `size`
* 由于消费者和生产者节奏不一致，队列常接近爆满或空。

## MPSC

Vyukov MPMC 思路，给每个槽位一个序号

```cpp

template <class T>
requires std::is_trivially_copyable_v<T>
struct ring {
    struct cell {
        std::atomic<size_t> seq;
        T data;
    };

    cell* buf;
    size_t cap;   // power-of-two
    size_t mask;  // cap - 1

    std::atomic<size_t> tail{0}; // producers
    std::size_t head{0};              // 1consumer

    ring(cell* storage, size_t capacity_pow2)
        : buf(storage), cap(capacity_pow2), mask(capacity_pow2 - 1) {
        for (std::size_t i = 0; i < cap; ++i)
            buf[i].seq.store(i, std::memory_order_relaxed);
    }

    // producer: try push once; false means full/contended
    bool try_push(const T& v) noexcept {
        const size_t pos = tail.fetch_add(1, std::memory_order_relaxed);
        cell& c = buf[pos & mask];

        // slot is writable iff seq == pos
        if (c.seq.load(std::memory_order_acquire) != pos) 
	        return false;

        c.data = v;
        // publish: readable iff seq == pos+1
        c.seq.store(pos + 1, std::memory_order_release);
        return true;
    }

    // consumer (single): try pop once
    bool try_pop(T& out) noexcept {
        const std::size_t pos = head;
        cell& c = buf[pos & mask];

        // slot is readable iff seq == pos+1
        if (c.seq.load(std::memory_order_acquire) != pos + 1) 
	        return false;
        
        out = c.data;

        // recycle: next time this cell is writable when pos advanced by cap
        c.seq.store(pos + cap, std::memory_order_release);
        head = pos + 1;
        return true;
    }
};

```

## MPMC

传统方法：
- 使用 CAS 保护 `tail` (写入端)
- 使用 CAS 保护 `size`
- 使用 spinlock 保护 `head` (读取端)