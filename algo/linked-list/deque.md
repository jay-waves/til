## 双端队列

双端队列, Double-ended Queue, 允许从两端插入和删除的队列, 操作两侧数据时复杂度为 $O(1)$.

类似 c++ deque 实现，一个数组指针表，外加多个固定大小的小数组。支持：
* `push_back()` O(1)
* `push_front()` O(1)
* `pop_back()` O(1)
* `pop_front()` O(1)
* `operator[]` O(1)

## 树状数组

https://leetcode.cn/discuss/post/3568520/shu-zhuang-shu-zu-xian-xing-ban-by-yukiy-4cup/