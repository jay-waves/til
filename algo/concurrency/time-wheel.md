## 接口

模拟时间片调度。
* `id schedule_after(duration, fn)` 延迟执行单次。
* `id schedule_every(duration, fn)` 周期任务
* `cancel(id)`
* `start(), stop()` 

```cpp
using clock = std::chrono::steady_clock;
using duration = clock::duration;
using task_fn = std::funciton<void()>;
using task_id = uint64_t; // requires comparable

class timer_wheel final {
public:
	
	timer_wheel(duration tick, size_t wheel_size, size_t levels = 4) :
		tick_(tick), 
		wheel_size_(wheel_size), 
		levels_(levels),
		wheels_(levels)
		{
		if (
			wheel_size == 0 || levels == 0 || tick.count() <= 0
		)	
			throw std::invalid_argument;
		
		for (auto&. lv: wheels_)
			lv.buckets.resize(wheel_size_);
	}
	
	~timer_wheel() { stop(); }
	void start();
	void stop();
	
	task_id schedule_after(duration d, task_fn fn);
	task_id schedule_every(duration period, task_fn fn);
	bool cancel(task_id id);

private:
	struct node {
		task_id id;
		task_fn fn;
		optional<duration> period;
		atomic<bool> cancelled = false;
		size_t rouunds = 0;
		shared_ptr<node> next;
	};
	
	struct bucket {
		shared_ptr<node> head;
	}

	duration tick_;
	size_t levels_;
	size_t wheel_size_;
	vector<level> wheels_;
	
	clock::time_point base_time_;
	clock::time_point next_tick_;
	uint64_t now_tick_; // 全局时钟
	
	mutex mu_;
	condition_variable cv_;
	atomic<bool> running_{false};
	thread worker_;
	
	clock::time_point base_time_{};
	clock::time_point next_tick_{};
	
	
	atomic<task_id> next_id_{1};
	std::unordered_map<std::uint64_t, std::weak_ptr<node>> index_;
};
```

## 底层时钟驱动

```cpp

```

```cpp


class timer_wheel final {
public:

  struct task_id {
    std::uint64_t v{};
    friend bool operator==(task_id a, task_id b) { return a.v == b.v; }
  };

  timer_wheel(duration tick, std::size_t wheel_size)
      : tick_(tick),
        wheel_size_(wheel_size),
        buckets_(wheel_size) {
    if (wheel_size_ == 0) throw std::invalid_argument("wheel_size must be > 0");
    if (tick_.count() <= 0) throw std::invalid_argument("tick must be > 0");
  }

  ~timer_wheel() { stop(); }

  timer_wheel(const timer_wheel&) = delete;
  timer_wheel& operator=(const timer_wheel&) = delete;

  void start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) return;

    {
      std::lock_guard lk(mu_);
      base_time_ = clock::now();
      next_tick_ = base_time_ + tick_;
    }
    worker_ = std::thread([this] { this->run_loop(); });
  }

  void stop() {
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false)) return;

    cv_.notify_all();
    if (worker_.joinable()) worker_.join();

    // 可选：清理剩余任务
    std::lock_guard lk(mu_);
    index_.clear();
    for (auto& b : buckets_) b.head = nullptr;
  }

  // 延迟执行一次
  task_id schedule_after(duration d, task_fn fn) {
    return schedule_impl(d, std::move(fn), /*period=*/std::nullopt);
  }

  // 周期执行（固定间隔；执行时间不计入漂移控制，本实现为“执行完再按 period 重新入轮”）
  task_id schedule_every(duration period, task_fn fn) {
    if (period.count() <= 0) throw std::invalid_argument("period must be > 0");
    return schedule_impl(period, std::move(fn), period);
  }

  bool cancel(task_id id) {
    std::lock_guard lk(mu_);
    auto it = index_.find(id.v);
    if (it == index_.end()) return false;
    auto node = it->second.lock();
    if (!node) {
      index_.erase(it);
      return false;
    }
    node->cancelled.store(true, std::memory_order_relaxed);
    index_.erase(it);
    return true;
  }

private:
  struct node final {
    task_id id{};
    task_fn fn{};
    std::optional<duration> period{};

    std::atomic<bool> cancelled{false};
    std::size_t rounds{0};

    std::shared_ptr<node> next{};
  };

  struct bucket final {
    std::shared_ptr<node> head{};
  };

  task_id schedule_impl(duration d, task_fn fn, std::optional<duration> period) {
    if (!fn) throw std::invalid_argument("fn is empty");
    if (d.count() < 0) d = duration::zero();

    const auto id = task_id{next_id_.fetch_add(1, std::memory_order_relaxed)};

    auto n = std::make_shared<node>();
    n->id = id;
    n->fn = std::move(fn);
    n->period = period;

    {
      std::lock_guard lk(mu_);
      insert_locked(n, d);
      index_[id.v] = n;
    }

    cv_.notify_all();
    return id;
  }

  void insert_locked(const std::shared_ptr<node>& n, duration d_from_now) {
    // 计算需要多少 tick
    const auto ticks = ceil_div(d_from_now, tick_);
    const auto slot_offset = static_cast<std::size_t>(ticks % wheel_size_);
    const auto rounds = static_cast<std::size_t>(ticks / wheel_size_);

    const std::size_t slot = (cursor_ + slot_offset) % wheel_size_;
    n->rounds = rounds;

    // push-front 单链表
    n->next = buckets_[slot].head;
    buckets_[slot].head = n;
  }

  static std::uint64_t ceil_div(duration a, duration b) {
    // a,b >= 0
    const auto ac = a.count();
    const auto bc = b.count();
    if (ac <= 0) return 0;
    // 注意：duration::count() 类型未必是整型，但 steady_clock::duration 通常是整型纳秒。
    // 为安全起见，转成 long double 再 ceil。
    long double x = static_cast<long double>(ac) / static_cast<long double>(bc);
    auto r = static_cast<std::uint64_t>(std::ceil(x));
    return r;
  }

  void run_loop() {
    std::unique_lock lk(mu_);
    while (running_.load(std::memory_order_relaxed)) {
      const auto now = clock::now();
      if (now < next_tick_) {
        cv_.wait_until(lk, next_tick_, [&] {
          return !running_.load(std::memory_order_relaxed) || clock::now() >= next_tick_;
        });
        continue;
      }

      // 赶 tick：如果卡顿，可能一次前进多格
      std::size_t steps = 0;
      const auto cur = clock::now();
      while (cur >= next_tick_) {
        next_tick_ += tick_;
        ++steps;
        if (steps > wheel_size_ * 4) break; // 防御：避免极端情况下无限追赶
      }

      for (std::size_t i = 0; i < steps; ++i) {
        tick_once_locked(lk);
        cursor_ = (cursor_ + 1) % wheel_size_;
      }
    }
  }

  void tick_once_locked(std::unique_lock<std::mutex>& lk) {
    auto& b = buckets_[cursor_];

    // 将 bucket 链表摘下，减少持锁时间
    auto list = std::exchange(b.head, nullptr);

    // 先分两类：到期执行、未到期回插
    std::vector<std::shared_ptr<node>> due;
    std::vector<std::shared_ptr<node>> reinsert;

    for (auto p = list; p;) {
      auto next = p->next;
      p->next = nullptr;

      if (p->cancelled.load(std::memory_order_relaxed)) {
        // 已取消：丢弃
      } else if (p->rounds > 0) {
        --p->rounds;
        reinsert.push_back(p);
      } else {
        due.push_back(p);
      }

      p = std::move(next);
    }

    // 回插未到期
    for (auto& n : reinsert) {
      // 仍然放回当前 slot（rounds 递减逻辑保证仍在这里等待下一圈）
      n->next = buckets_[cursor_].head;
      buckets_[cursor_].head = n;
    }

    // 执行到期任务：解锁执行，避免用户 fn 阻塞时间轮
    lk.unlock();
    for (auto& n : due) {
      if (n->cancelled.load(std::memory_order_relaxed)) continue;

      try {
        n->fn();
      } catch (const std::exception& e) {
        std::cerr << "[timer_wheel] task threw: " << e.what() << "\n";
      } catch (...) {
        std::cerr << "[timer_wheel] task threw (unknown)\n";
      }

      // 周期任务：重新调度
      if (n->period.has_value() && !n->cancelled.load(std::memory_order_relaxed)) {
        std::lock_guard g(mu_);
        insert_locked(n, *n->period);
        // index_ 中仍然保留该 id -> node 的弱引用，不需要更新
      } else {
        // 一次性任务：从索引里清掉（可能 cancel 已经清了，这里再清理一遍无妨）
        std::lock_guard g(mu_);
        index_.erase(n->id.v);
      }
    }
    lk.lock();
  }

private:
  duration tick_;
  std::size_t wheel_size_;
  std::vector<bucket> buckets_;

  std::mutex mu_;
  std::condition_variable cv_;

  std::atomic<bool> running_{false};
  std::thread worker_;

  clock::time_point base_time_{};
  clock::time_point next_tick_{};

  std::size_t cursor_{0};

  std::atomic<std::uint64_t> next_id_{1};

  // id -> node (weak 避免循环引用；bucket/临时执行向量持 shared_ptr)
  std::unordered_map<std::uint64_t, std::weak_ptr<node>> index_;
};
```