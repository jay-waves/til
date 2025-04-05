#pragma once
#include <utility>  // For std::swap

template <typename T>
class cow_ptr {
 public:
  explicit cow_ptr(T* ptr = nullptr) : ptr_(ptr), ref_cnt_(new int(1)) {}
  ~cow_ptr() { release(); }

  cow_ptr(const cow_ptr& other) : ptr_(other.ptr_), ref_cnt_(other.ref_cnt_) {
    ++(*ref_cnt_);
  }
  cow_ptr& operator=(const cow_ptr& other) {
    if (this != &other) {
      release();
      ptr_ = other.ptr_;
      ref_cnt_ = other.ref_cnt_;
      ++(*ref_cnt_);
    }
    return *this;
  }

  cow_ptr(cow_ptr&& other) noexcept : ptr_(other.ptr_), ref_cnt_(other.ref_cnt_) {
    other.ptr_ = nullptr;
    other.ref_cnt_ = nullptr;
  }
  cow_ptr& operator=(cow_ptr&& other) noexcept {
    if (this != &other) {
      release();
      ptr_ = other.ptr_;
      ref_cnt_ = other.ref_cnt_;
      other.ptr_ = nullptr;
      other.ref_cnt_ = nullptr;
    }
    return *this;
  }

  T& operator*() {
    ensure_unique();
    return *ptr_;
  }
  T* operator->() {
    ensure_unique();
    return ptr_;
  }

  // Get the raw pointer
  const T* get() const { return ptr_; }

  // Get the reference count
  int use_count() const { return (ref_cnt_ != nullptr) ? *ref_cnt_ : 0; }

  // Check if the pointer is unique (i.e., only one cow_ptr owns the object)
  bool unique() const { return use_count() == 1; }

  // Reset the managed object
  void reset(T* ptr = nullptr) {
    release();
    ptr_ = ptr;
    ref_cnt_ = new int(1);
  }

 private:
  // Ensure the object is uniquely owned before modification
  void ensure_unique() {
    if (!unique()) {
      --(*ref_cnt_);
      ptr_ = ptr_ ? new T(*ptr_) : nullptr;
      ref_cnt_ = new int(1);
    }
  }

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

