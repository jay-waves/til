## 有锁队列

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

## 无锁队列（Michael-Scott Queue）

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

比较公认的无锁修复方案是 hazard pointer 。这里没看实现（不想看了，不行就用 GC 语言吧）

```cpp
Node *p;
do {
	p = head.load(std::memory_order_acquire);
	// 这里 p 还是可能被释放，while() 条件中再检查一次
	my_hazrd.store(p, std::memory_order_release);
} while (p != head.load(std::memory_order_acqurie));
```