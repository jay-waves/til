## 有锁队列

高性能队列，通常使用 **RingBuffer** 当作底层数据结构。

为什么 Producer-Consumer 问题不适宜使用链式队列？
* 显式维护 `size`
* 由于消费者和生产者节奏不一致，队列常接近爆满或空。
* 多线程在 `head, tail` 上的频繁竞争
* 队列常使用链式结构，对缓存不友好

### MPMC (Multi Producers, Multi Consumers)

传统方法：
- 使用 CAS 保护 `tail` (写入端)
- 使用 CAS 保护 `size`
- 使用 spinlock 保护 `head` (读取端)
- `size` 使用 power_of_2 , 来减少取余操作的开销

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

## 无锁队列

### Michael-Scott Queue

* 基于 Linked List ，不固定容量
* 采用 CAS 操作无锁推进 head 和 tail 指针。并且 CAS 操作的重试次数有限。
* Lock-Free （至少一个线程能前进）


```cpp
template <typename T>
class LKQueue {
private:
    struct Node {
        std::optional<T> value; // or shared_ptr<T>
        std::atomic<Node*> next;

        Node() : value(std::nullopt), next(nullptr) {}

        explicit Node(T v)
            : value(std::move(v)), next(nullptr) {}
    };

    std::atomic<Node*> head;
    std::atomic<Node*> tail;

public:
	LKQueue();
	~LKQueue();
    LKQueue(const LKQueue&) = delete;
    LKQueue& operator=(const LKQueue&) = delete;

	void enqueue(T value); 
	std::optional<T> dequeue();
};
```

```cpp

LKQueue::LKQueue() {
	Node* dummy = new Node();
	head.store(dummy, std::memory_order_relaxed);
	tail.store(dummy, std::memory_order_relaxed);
}

LKQueue::~LKQueue() {
	while (dequeue().has_value()) {}

	Node* dummy = head.load(std::memory_order_relaxed);
	delete dummy;
}


void LKQueue::enqueue(T value) {
	Node* n = new Node(std::move(value));

	while (true) {
		Node* last = tail.load(std::memory_order_acquire);
		Node* next = last->next.load(std::memory_order_acquire);

		if (last == tail.load(std::memory_order_acquire)) {
			if (next == nullptr) {
				if (last->next.compare_exchange_weak(
						next,
						n,
						std::memory_order_release,
						std::memory_order_relaxed)) {
					tail.compare_exchange_weak(
						last,
						n,
						std::memory_order_release,
						std::memory_order_relaxed);
					return;
				}
			} else {
				tail.compare_exchange_weak(
					last,
					next,
					std::memory_order_release,
					std::memory_order_relaxed);
			}
		}
	}
}

std::optional<T> LKQueue::dequeue() {
	while (true) {
		Node* first = head.load(std::memory_order_acquire);
		Node* last = tail.load(std::memory_order_acquire);
		Node* next = first->next.load(std::memory_order_acquire);

		if (first == head.load(std::memory_order_acquire)) {
			if (first == last) {
				if (next == nullptr) {
					return std::nullopt;
				}

				tail.compare_exchange_weak(
					last,
					next,
					std::memory_order_release,
					std::memory_order_relaxed);
			} else {
				T value = std::move(*(next->value));

				if (head.compare_exchange_weak(
						first,
						next,
						std::memory_order_release,
						std::memory_order_relaxed)) {
					delete first;
					return value;
				}
			}
		}
	}
}
```

#### hazard pointer 

上述代码有一个 BUG：

```cpp
	Node* last = tail.load(std::memory_order_acquire);
	// 其他线程释放了 last 
	Node* next = last->next.load(std::memory_order_acquire); // UAF 
```

比较公认的无锁修复方案是 hazard pointer 。实现比较复杂，这里略。

```cpp
Node *p;
do {
	p = head.load(std::memory_order_acquire);
	// 这里 p 还是可能被释放，while() 条件中再检查一次
	my_hazrd.store(p, std::memory_order_release);
} while (p != head.load(std::memory_order_acqurie));
```

在 GC 语言中，内存释放被托管给了 GC 处理，因此规避了悬空指针的问题。

#### ABA Problem 


### Vyukov MPMC

Vyukov MPMC 思路，用额外的 seq 空间，来规避在 `head, tail` 上的全局竞争。
* 基于 RingBuffer 环形缓冲区，固定容量
* 基于版本号索引，减少 CAS 重试，优化缓存
* Wait-Free （有上界无自旋无限循环，比 Lock-free 稍弱）

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

    alignas(64) std::atomic<size_t> tail{0}; // producer pos 
    alignas(64) std::atomic<size_t> head{0}; // consumer pos
};
```

初始时，`seq = i`

```cpp
ring(cell* storage, size_t capacity_pow2)
        : buf(storage), cap(capacity_pow2), mask(capacity_pow2 - 1) {
        for (std::size_t i = 0; i < cap; ++i)
            buf[i].seq.store(i, std::memory_order_relaxed);
    }
```

不断递增全局 `pos`，用 `i = pos & mask` 定位到某个槽。当看到 `seq == pos` 时，通过 `seq++` 来将其标记为可读。同时屏蔽其他写者。

```cpp
// producer: try push once; false means full/contended
bool try_push(const T& v) noexcept {
    cell* c;
    size_t pos = tail.load(std::memory_order_relaxed);

    for (;;) {
        c = &buf[pos & mask];

        size_t seq = c->seq.load(std::memory_order_acquire);
        intptr_t dif = static_cast<intptr_t>(seq) - static_cast<intptr_t>(pos);

        if (dif == 0) {
            if (tail.compare_exchange_weak(
                    pos,
                    pos + 1,
                    std::memory_order_relaxed,
                    std::memory_order_relaxed)) {
                break; // 抢到这个 pos，对应 cell 归当前 producer
            }
        } else if (dif < 0) {
            return false; // 队列满
        } else {
            pos = tail.load(std::memory_order_relaxed);
        }
    }

    c->data = v;

    c->seq.store(pos + 1, std::memory_order_release);
    return true;
}
```

消费者看到 `seq == pos + 1` 时，读取数据。然后将 `seq = pos + N`，允许下一轮回绕到这个槽的生产者写入。

```cpp
// consumer: try pop once
bool try_pop(T& out) noexcept {
    cell* c;
    size_t pos = head.load(std::memory_order_relaxed);

    for (;;) {
        c = &buf[pos & mask];

        size_t seq = c->seq.load(std::memory_order_acquire);
        intptr_t dif = static_cast<intptr_t>(seq) - static_cast<intptr_t>(pos + 1);

        if (dif == 0) {
            if (head.compare_exchange_weak(
                    pos,
                    pos + 1,
                    std::memory_order_relaxed,
                    std::memory_order_relaxed)) {
                break; // 抢到这个 pos，对应 cell 归当前 consumer
            }
        } else if (dif < 0) {
            return false; // 队列空
        } else {
            pos = head.load(std::memory_order_relaxed);
        }
    }

    out = c->data;

    c->seq.store(pos + mask + 1, std::memory_order_release);
    return true;
}
```

#### Cache Line Padding 

实践中，还会分离读者和写者的版本号。这是为了，在队列饱满或者空（并不罕见）时，进一步减少生产者、消费者之间的竞争。（持有同一个 cell 导致 seq 争用）

```cpp
struct cell {
    alignas(64) std::atomic<size_t> produce_seq; // 生产者写，消费者读
    alignas(64) std::atomic<size_t> consume_seq; // 消费者写，生产者读
    T data_;
}; // C++20
```

## 参考

2011 -- Thompson, M. -- Disruptor, High performance alternative to bounded queues for exchanging data between concurrent threads


[linux kernel kfifo](../linked-list/queue.md)