* `touch(k)` 更新 `k` 的访问时间。访问时间通过双端链表 `list<Key>` 维护，为保证访问速度，用 `unordered_map<K, iterator>` 记录索引。
* 当容量不足时，弹出最近最少使用的键。

```cpp
template <class Key>
class LruSet {
public:
    explicit LruSet(std::size_t cap) : cap_(cap) {}

    void touch(const Key& key) {
	    auto it = pos_.find(key);
        if (it != pos_.end()) {
	        // move to the front
            order_.splice(order_.begin(), order_, it->second);
            return;
        }

        order_.push_front(key);
        pos_[key] = order_.begin();

        if (order_.size() > cap_) {
            pos_.erase(order_.back());
            order_.pop_back();
        }
    }

private:
    std::size_t cap_;
    list<Key> order_; // LRU
    unordered_map<Key, typename list<Key>::iterator> pos_;
};
```