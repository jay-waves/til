
参数：
* *核心线程数*：任务队列未到达容量时，稳定的线程数，不会轻易销毁。
* *最大线程数*：任务队列达到容量上限时，扩容线程数
* *存活时间*：扩容的线程的存活时间
* *线程取消与退出令牌*：
* *任务*：Callable 接口。可能有状态机（提交、执行、错误、完成）
* *任务队列*：用于存储等待执行的任务

### Work Stealing Thread Pool 

简单的线程安全队列。后续每个工作线程会拥有一个 `jobs_queue` ，拥有者通过 `pop_front()` 获取任务，其他空闲线程通过 `steal_back()` 窃取任务

```cpp
using job_t = std::function<void()>;

class jobs_queue {
public:
    jobs_queue() = default;

    void push_front(job_t job) {
        std::lock_guard<std::mutex> lk(mu_);
        q_.push_front(std::move(job));
    }

    optional<job_t> pop_front() {
        std::lock_guard<std::mutex> lk(mu_);

        if (q_.empty()) return nullopt;

        job_t job = std::move(q_.front());
        q_.pop_front();
        return job;
    }

    optional<job_t> steal_back() {
        std::lock_guard<std::mutex> lk(mu_);
        if (q_.empty()) return nullopt;

        job_t job = std::move(q_.back());
        q_.pop_back();
        return job;
    }

private:
    std::mutex mu_;
    std::deque<job_t> q_;
};
```

简易线程池接口：

```cpp
class thread_pool_t {
public:
    virtual void wait_all() = 0;
    virtual void submit(job_t) = 0;
};

class thread_pool : public thread_pool_t {
public:
    explicit thread_pool(size_t n_threads);

    ~thread_pool();

    void submit(job_t job) override;

    void wait_all() override;

private:
    void complete_n(size_t n_done);

    bool try_take_local(size_t tid, job_t& job);

    bool try_steal(size_t tid, job_t& job);

    void worker_loop(std::stop_token st, size_t tid);

private:
    size_t n_threads_;
    std::atomic<uint64_t> rr_;

    std::atomic<uint64_t> pending_; // jobs uncompleted (including activated jobs)
    std::atomic<uint64_t> queued_;  // jobs detained in queues (without activated jobs)

    std::mutex work_mu_;
    std::condition_variable work_cv_;

    std::mutex done_mu_;
    std::condition_variable done_cv_;

    deque<jobs_queue> queues_;
    vector<std::jthread> workers_;
};
```

工作线程循环：

```cpp
thread_pool::thread_pool(size_t n_threads) 
        : n_threads_(n_threads),
          rr_(0),
          pending_(0),
          queued_(0) {

	queues_.resize(n_threads_); 
	workers_.reserve(n_threads_);

	for (size_t tid = 0; tid < n_threads_; ++tid) {
		workers_.emplace_back( 
				[this, tid](std::stop_token st) {  worker_loop(st, tid);  }
		);
	}
}

inline constexpr auto mo_relaxed = std::memory_order_relaxed;
inline constexpr auto mo_acquire = std::memory_order_acquire;
inline constexpr auto mo_release = std::memory_order_release;
inline constexpr auto mo_acq_rel = std::memory_order_acq_rel;

void thread_pool::worker_loop(std::stop_token st, size_t tid) {
	job_t job; // local arena

	for (;;) {
		if (!try_take_local(tid, job) && !try_steal(tid, job)) {
			
			std::unique_lock<std::mutex> lk(work_mu_);
			work_cv_.wait(lk, [&] {
					return st.stop_requested() || queued_.load(mo_acquire) > 0;
			});

			if (st.stop_requested() && pending_.load(mo_acquire) == 0) {
				return;
			}
			continue;
		}

		job(); // try catch?
		complete_n(1);
	}
}
	
void complete_n(size_t n_done) {
	if (n_done == 0) return;
	const auto old = pending_.fetch_sub(n_done, mo_acq_rel);
	if (old == n_done) {
		done_cv_.notify_all();
	}
}

bool try_take_local(size_t tid, job_t& job) {
	if (auto local = queues_[tid].pop_front()) {
		job = std::move(*local);
		queued_.fetch_sub(1, mo_acq_rel);
		return true;
	}
	return false;
}

bool try_steal(size_t tid, job_t& job) {
	if (n_threads_ <= 1) return false;

	for (size_t k = 1; k < n_threads_; ++k) {
		const auto victim = (tid + k) % n_threads_;
		if (auto stolen = queues_[victim].steal_back()) {
			job = std::move(*stolen);
			queued_.fetch_sub(1, mo_acq_rel);
			return true;
		}
	}
	return false;
}
	
thread_pool::~thread_pool() {
	for (auto& t : workers_) t.request_stop();

	work_cv_.notify_all();
	done_cv_.notify_all();
}
```

提交任务与等待：

```cpp
void thread_pool::submit(job_t job) override {
	pending_.fetch_add(1, mo_relaxed);
	queued_.fetch_add(1, mo_release);

	const auto tid = rr_.fetch_add(1, mo_relaxed) % n_threads_;
	queues_[tid].push_front(std::move(job));

	// 全局唤醒
	work_cv_.notify_one();
}

void thread_pool::wait_all() override {
	std::unique_lock<std::mutex> lk(done_mu_);
	done_cv_.wait(lk, [&] { return pending_.load(mo_acquire) == 0; });
}
```

### 其他问题

lambda 函数可能造成频繁堆内存分配，避免 `[]` 捕获大对象

false sharing 问题：

```cpp 
vec_t local_sum(n_threads); 
// 每个线程的本地部分，由于没有填充到对齐 cache line，存在核之间的伪共享。
// 比如，一个线程频繁更新 local_sum[0]，另一个线程频繁读取 local_sum[1]，则
// 这两个变量可能位于同一个 cache line 上，导致频繁的缓存失效。

for (size_t tid = 0; tid < n_threads; ++tid) {

	pool.submit([&, tid, begin, end] {
		local_sum[tid] += ...;
	});
}
```

缓存友好版本：

```cpp
struct alignas(64) local_sum {
	double v = 0.0;
};
std::vector<local_sum> locals(n_threads);
// 每个线程的本地部分，使用 alignas(64) 保证每个 local_sum 独占一个缓存行，
// 避免伪共享带来的缓存一致性开销。
```

任务批处理：指任务批量出队，工作线程一次可以从任务队列中获取多个小任务。降低整体锁竞争次数、降低线程唤醒（操作系统调用）次数。