STL 是 C++ 标准库的一部分, 提供一系列模板化的数据结构和算法. 支持泛型, 高度性能优化.

主要部分为:
1. 容器, Containers
	- [顺序容器](sequences.md): 存储序列, 允许双向遍历. `vector`, `list`, `deque`, `array`, `forward_list`
	- [关联容器](associative%20array.md): 存储键值对. `map`, `multimap`, `set`, `multiset`
	- 散列容器: 即无序关联容器 `unordered_map`, `unordered_set`, `unordered_multimap`, `unordered_multiset`
	- [容器适配器](adapters.md): `queue`, `priority_queue`, `stack`, `bitset`
3. 算法, Alogirithms
4. 迭代器, Iterators
5. 函数对象, Funciton Objects
6. 适配器, Adapters, 将某种容器的接口适配成另一种容器.

pair, span, optional, variant, any

|          | 关联容器           | 散列容器                              | 顺序容器 |
| -------- | ------------------ | ------------------------------------- | -------- |
| 顺序     | 有序, 按键值排序   | 无序                                  | 有序, 按插入顺序         |
| 实现方式 | 红黑树             | 哈希表                                | 数组或双向链表         |
| 查找时间 | $O(\log n)$        | 平均 $O(1)$, 最坏 $O(n)$              |          |
| 插入时间 |                    |                                       |          |
| 删除时间 |                    |                                       |          |
| 空间     |  高                  | 高 (存储哈希表, 同时预留空间防止碰撞) |          |
| 场景     | 有序存储, 范围查询 | 查找和更新速度                        |          |

## 容器

## 常见操作方法

| 所属容器类型 | 原型                                                                                                | 作用                                   |                                            |
| ------------ | --------------------------------------------------------------------------------------------------- | -------------------------------------- | ------------------------------------------ |
| 通用         | `size_type size() const noexcept`                                                                   | 容器中元素个数                         |                                            |
|              | `size_type capacity() const noexcept`                                                               | 返回容器当前分配的空间可容纳的元素数量 | 仅 `vector, string`                        |
|              | `void resize(size_type count, /* const value_type& value */)`                                       | 修改大小, 并用 `value` 填充            |                                            |
|              | `bool empty() const noexcept`                                                                       | 判断是否为空                           |                                            |
|              | `void clear() noexcept`                                                                             | 清空容器中所有元素                     |                                            |
|              | `iterator begin() noexcept`, `end()`                                                                | 用于迭代                               |                                            |
|              | `iterator insert(iterator pos, const T& /*T&&*/ value)`                                             | 指定位置插入元素                       | `vector` 可能重新分配内存                  |
|              | `iterator erase(iterator pos)`, `erase(iterator first, last)`                                       | 移除几个元素                           | `vector, deque` 可能导致迭代器失效         |
|              | `void swap(Contianer& other) noexcept`                                                              | 交换容器内存                           |                                            |
|              | `reference at(size_type pos)`                                                                       | 返回指定位置的元素                     | 有越界检查, 抛出 `out_of_range` 异常       |
|              | `T* data noexcept`                                                                                  | 直接获取存储元素数组的指针             | 仅 `vector, string, array`                 |
|              | `iterator emplace(const_iterator pos, Args&&... agrs)`                                              | 在指定位置原地构造元素                 | `vector` 可能导致内存分配                  |
|              | `shrink_to_fit()`                                                                                   | 适当缩减当前容器大小                   | 仅 `vecotr, string, deque`, 可能迭代器失效 |
| 仅顺序容器   | `void push_back(const T& /* T&& */ value)`                                                          | 在容器末尾添加元素                     | `vector` 可能重新分配内存                  |
|              | `void pop_back()`                                                                                   | 移除容器末尾元素                       |                                            |
|              | `void push_front(const T& value)`                                                                   | 在开头插入元素                         | 仅 `deque`                                  |
|              | `void pop_front()`                                                                                  | 移除开头元素                                       |                                            |
|              | `T& front()`                                                                                        | 返回容器第一个元素                     |                                            |
|              | `T& back()`                                                                                         | 返回容器最后的元素                     |                                            |
| 关联容器     | `iterator insert(iterator hint, const value_type& value)`, </br> `pair<iterator, bool> insert(...)` | 插入元素, 如果已存在则无动作           |                                            |
|              | `iterator find(const Key& key)`                                                                     | 在关联容器中查找元素                   |                                            |
|              | `size_type count(const Key& key) const`                                                             | 返回键为 `key` 的元素个数              |                                            |
|              | `iterator lower_bound(const Key& key)`                                                              | 返回第一个不小于 `key` 的迭代器        |                                            |
|              | `upper_bound(...)`                                                                                  | 返回第一个大于 `key` 的迭代器          |                                            |
| 无序容器     | `size_type bucket_count() const`                                                                    | 返回容器中桶的数量                     | 用于无序关联容器                           |
|              |                                                                                                     |                                        |                                            |

