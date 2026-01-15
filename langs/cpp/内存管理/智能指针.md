| `unique_ptr` | `shared_ptr` |
| ------------ | ------------ |
| `type&`             |  `weak_ptr`          |

推荐在处理多态对象时, 使用 `unique_ptr`; 在共享对象时, 使用 `shared_ptr`; 其他情况尽量使用 STL 容器, 因为更简洁有效.

## unique_ptr

```cpp
template <typename T>
class smart_ptr {
public:
	explicit smart_ptr(T* ptr = nullptr) : ptr_(ptr) {}
	~smart_ptr() {
		delete ptr_;
	}
	T* get() const {return ptr_; }
	T& operator * () const { return *ptr_; }
	T* operator -> () const { return *ptr_; }
	operator bool() const { return ptr_; }

	// 拷贝时不应该改变原指针, 默认禁用.
	
	// 移动时可以交出所有权. U 是 T 的多态.
	template <typename U>
	smart_ptr(smart_ptr<U>&& other) noexcept {
		ptr_ = other.release();
	}
	template <typename U>
	smart_ptr& operator=(smart_ptr<U> rhs) noexcept {
		rhs.swap(*this);
		return *this;
	}
private:
	T* ptr_;
	T* release() noexcept {
		T* ptr = ptr_;
		ptr_ = nullptr;
		return ptr;
	}
	void swap(smart_ptr& rhs) noexcept {
		 using std::swap;
		 swap(ptr_, rhs.ptr_);
	}
};
```

## shared_ptr

```cpp
// 不考虑线程安全, 不考虑循环引用
template <typename T>
class shared_ptr {
public:
	explicit shared_ptr(T* ptr = nullptr) 
			: ptr_(ptr), ref_count(new size_t(ptr ? 1 : 0)) {}
	
	// 演示拷贝构造和移动构造. 赋值操作不演示
	shared_ptr(const shared_ptr& other)
			: ptr_(other.ptr_), ref_count_(other.ref_count_) {
		if (ptr_)
			++(*ref_count_);
	}
	shared_ptr(shared_ptr&& other) noexcept 
			: ptr_(other.ptr_), ref_count_(other.ref_count_) {
		other.ptr_ = nullptr;
		other.ref_count_ = nullptr;
	}
	
	~shared_ptr() { release(); }
	
	T& operator*() const { return *ptr_; }
	T* operator->() const { return ptr_; }
	size_t use_count() const {return ptr_ ? *ref_count_ : 0; }
	bool unique() const { return use_count() == 1 }

private:
	void release() {
		if (ptr_)
			if (--(*ref_count_ ) == 0) {
				delete ptr_;
				delete ref_count_;
			}
		
		ptr_ = nullptr;
		re_count_ = nullptr;
	}
	
	T* ptr_;
	size_t * ref_count_;
}

```

为了避免 `shared_ptr` 循环引用导致的"资源泄漏", 标准库引入 `weak_ptr`. 在对一个资源的控制块 (Control Block) 中, 维护:
- 强引用计数, strong_count. 指多少个 `shared_ptr` 持有对象.
- 弱引用计数, weak_count. 指多少个 `weak_ptr` 持有对象. 
- 对于某对象, 必须要有一个 `shared_ptr` 存在, 才能创建 `weak_ptr`.
- 当 `strong_count == 0` 时, 销毁对象
- 当 `strong_count == 0 && weak_count == 0`, 销毁控制块自身. 也就是说, `weak_ptr` 不影响对象的生命周期, 但能检测对象的状态.

```cpp
// 用 atomic 保证线程安全
struct ctl_block {
	std::atomic<size_t> strong_count{1}; 
	std::atomic<size_t> weak_count{0};
	void* obj;
};
```

## cow_ptr

```cpp
template <typename T>
class cow_ptr {
public:
	cow_ptr() : ptr_(std::make_shared<T>()) {}
	explicit cow_ptr(const T& value) 
			: ptr_(std::make_shared<T>(value)) {}
	explicit cow_ptr(T&& value) 
			: ptr_(std::maek_shared<T>(std::move(value))) {}
	
	// 读访问
	const T& operator*() const { return *ptr_; }
	const T* operator->() const { return ptr_.get(); }
	
	// 写访问 (非 const 引用): 确保唯一所有权, 否则拷贝
	T* operator->() { return &detach(); }
	T& operator*() { return detach(); }

private:
	T& detach() {
		if (!ptr_.unique()) 
			ptr_ = std::make_shared<T>(*ptr_);
		return *ptr_;
	}
	std::shared_ptr<T> ptr_;
};
```