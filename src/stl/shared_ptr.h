#pragma once
/*
 * shared_ptr, weak_ptr, cow_ptr
 */

template <typename T>
class shared_ptr {
 public:
  explicit shared_ptr(T* ptr = nullptr) : ptr_(ptr), ref_cnt_(new int(1)) {}

  ~shared_ptr() { release(); }

  shared_ptr(const shared_ptr& other) : ptr_(other.ptr_), ref_cnt_(other.ref_cnt_) {
    ++(*ref_cnt_);
  }

  shared_ptr& operator=(const shared_ptr& other) {
    if (this != &other) {
      release();
      ptr_ = other.ptr_;
      ref_cnt_ = other.ref_cnt_;
      ++(*ref_cnt_);
    }
    return *this;
  }

  shared_ptr(shared_ptr&& other) noexcept : ptr_(other.ptr_), ref_cnt_(other.ref_cnt_) {
    other.ptr_ = nullptr;
    other.ref_cnt_ = nullptr;
  }

  shared_ptr& operator=(shared_ptr&& other) noexcept {
    if (this != &other) {
      release();
      ptr_ = other.ptr_;
      ref_cnt_ = other.ref_cnt_;
      other.ptr_ = nullptr;
      other.ref_cnt_ = nullptr;
    }
    return *this;
  }

  // Dereference operators
  T& operator*() const { return *ptr_; }
  T* operator->() const { return ptr_; }

  // Get the raw pointer
  T* get() const { return ptr_; }

  // Get the reference count
  int use_count() const { return (ref_cnt_ != nullptr) ? *ref_cnt_ : 0; }

  // Check if the pointer is unique (i.e., only one shared_ptr owns the object)
  bool unique() const { return use_count() == 1; }

  // Reset the managed object
  void reset(T* ptr = nullptr) {
    release();
    ptr_ = ptr;
    ref_cnt_ = new int(1);
  }

 private:
  // Release the managed object
  void release() {
    if (ref_cnt_ && --(*ref_cnt_) == 0) {
      delete ptr_;
      delete ref_cnt_;
    }
    ptr_ = nullptr;
    ref_cnt_ = nullptr;
  }

  T* ptr_;
  int* ref_cnt_;
};

template <typename T>
class weak_ptr {
 public:
  weak_ptr() : ptr_(nullptr), ref_count_(nullptr) {}

  explicit weak_ptr(const shared_ptr<T>& shared_ptr) : ptr_(shared_ptr.ptr_), ref_count_(shared_ptr.ref_count_) {}

  weak_ptr(const weak_ptr& other) : ptr_(other.ptr_), ref_count_(other.ref_count_) {}
  weak_ptr& operator=(const weak_ptr& other) {
    if (this != &other) {
      ptr_ = other.ptr_;
      ref_count_ = other.ref_count_;
    }
    return *this;
  }

  weak_ptr(weak_ptr&& other) noexcept : ptr_(other.ptr_), ref_count_(other.ref_count_) {
    other.ptr_ = nullptr;
    other.ref_count_ = nullptr;
  }
  weak_ptr& operator=(weak_ptr&& other) noexcept {
    if (this != &other) {
      ptr_ = other.ptr_;
      ref_count_ = other.ref_count_;
      other.ptr_ = nullptr;
      other.ref_count_ = nullptr;
    }
    return *this;
  }

  // Lock the weak_ptr to obtain a shared_ptr
  shared_ptr<T> lock() const {
    return (expired() ? shared_ptr<T>(nullptr) : shared_ptr<T>(*this));
  }

  // Check if the referenced object has been deleted
  bool expired() const {
    return (ref_count_ == nullptr || *ref_count_ == 0);
  }

 private:
  T* ptr_;
  int* ref_count_;
};

