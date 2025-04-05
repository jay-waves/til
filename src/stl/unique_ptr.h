#pragma once

template <typename T>
class unique_ptr {
 public:
  explicit unique_ptr(T* ptr = nullptr) : ptr_(ptr) {}
  ~unique_ptr() { delete ptr_; }

  // Move Constructor
  unique_ptr(unique_ptr&& other) noexcept : ptr_(other.ptr_) {
    other.ptr_ = nullptr;
  }

  // Move Assignment
  unique_ptr& operator=(unique_ptr&& other) noexcept {
    if (this != &other) {
      delete ptr_;
      ptr_ = other.ptr_;
      other.ptr_ = nullptr;
    }
    return *this;
  }

  // Disable Copy Constructor and Copy Assignment
  unique_ptr(const unique_ptr&) = delete;
  unique_ptr& operator=(const unique_ptr&) = delete;

  // Dereference operators
  T& operator*() const { return *ptr_; }
  T* operator->() const { return ptr_; }

  T* get() const { return ptr_; }

 private:
  T* ptr_;
};

template<typename T, typename... Args>
unique_ptr<T> make_unique(Args&&... args) {
	return std::unique_ptr<T>{ new T{std::forward<Args>(args)...} };
}

