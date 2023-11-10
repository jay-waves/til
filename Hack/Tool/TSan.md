`ThreadSanitizer` (aka. TSan) detects data-race bugs for c. **Data race** , which is banned by C++11 as *Undefined Behavior*,  occurs when two threads access the same variable concurrently and at least one of the accesses is write.

*use*: compile with flags `-fsanitizer=thread`.
- `-o2` optimize code performance.
- `-g` get file names and line numbers in the warning msgs.

## popular data-race cases

### 1 Simple race

**Thread-hostile reference counting**, 引用计数时, 没有互斥访问.

```cpp
// Ref() and Unref() may be called from several threads.
// Last Unref() destroys the object.
class RefCountedObject {
 ...
 public:
  void Ref() {
    ref_++;  // Bug!
  }
  void Unref() {
    if (--ref_ == 0)  // Bug! Need to use atomic decrement!
      delete this;
  }
 private:
  int ref_;
};
```

### 2 Race on a complex object

two threads access a non-thread-safe complex object (e.g. an STL container) without synchronization.

```cpp
std::map<int,int> m;

void Thread1() {
  m[123] = 1;
}

void Thread2() {
  m[345] = 0;
}
```

### 3 Publishing objects without synchronization