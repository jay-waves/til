```cpp
#pragma once
/*
  后台任务处理线程. 
  多个任务发生时, 只有一个后台工作线程, 按 FIFO 队列依次处理任务.
*/

#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <functional>

using namespace std;

class TaskQueue {
public:
  TaskQueue() {
    worker_ = thread(&TaskQueue::run, this);
  }

  TaskQueue(const TaskQueue&) = delete;
  TaskQueue& operator=(const TaskQueue&) = delete;
  TaskQueue(TaskQueue&&) = delete;
  TaskQueue& operator=(TaskQueue&&) = delete;

  ~TaskQueue() {
    {
      lock_guard<mutex> lock(mtx_);
      should_exit_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

  void addTask(const function<void()>& task) {
    {
      lock_guard<mutex> lock(mtx_);
      tasks_.push(task);
    }
    cv_.notify_one();
  }

  template<typename F, typename... Args>
  void addTask(F&& f, Args&&... args) {
      auto task = bind(forward<F>(f), forward<Args>(args)...);
      addTask(task);
  }

  void run() {
    while (true) {
      Task task;
      {
        unique_lock<mutex> lck(mtx_);
        cv_.wait(lck, [this]() {
          return should_exit_ || !tasks_.empty(); 
        });
        if (should_exit_ && tasks_.empty()) return;
        task = move(tasks_.front());
        tasks_.pop();
      }
      task(); 
    }
  }
private:
  using Task = function<void()>;
  queue<Task> tasks_;
  mutex mtx_; 
  condition_variable cv_; 
  thread worker_;
  bool should_exit_ = false;
};
```