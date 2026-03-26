运行时死锁检测方法：区分 S/X 节点，进行全局锁依赖图的环检测。

```cpp 
enum class lock_mode : std::uint8_t {

};

```