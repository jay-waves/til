## 桶排序（bucket sort）

把数据按值域的区间分配到若干桶内，然后桶内独立排序，再依次合并。须要数据分布比较均匀，或者说，桶映射函数的结果分布均匀，且值域已知；在极端集中的数据场景中，复杂度退化为 $O(n^{2})$ 。


## 基数排序（radix sort）

元素有多关键字，并且不便直接比较大小时，*基数排序* 按“关键字位”逐位进行排序。基数排序，行为类似多次桶排序。

基数排序的缺点：
* 继承桶排序缺点：
	* 至少一倍的额外内存消耗，且值域需要已知。 
	* 数据分布均匀时，入桶过程，约等于伪随机写。存在明显**缓存抖动**。
* 多次桶排序中，**内存拷贝瓶颈**大于 CPU 瓶颈。数据量明显比 L3 缓存大时，基数排序才有优势。

```cpp
// 按字节做基数排序
void radix_sort(vector<uint32_t>& a)
{
    const size_t n = a.size();
    if (n < 2) return;

    vector<uint32_t> buf(n);

    constexpr size_t RADIX = 256;   // 1B
    constexpr size_t PASSES = 4;    // 32b / 8

    for (size_t pass = 0; pass < PASSES; ++pass) {
        size_t count[RADIX] = {0}; // 基数直方图

        const size_t shift = pass * 8;

        for (size_t i = 0; i < n; ++i) {
            uint8_t key = (a[i] >> shift) & 0xFF;
            ++count[key];
        }

        size_t pos[RADIX] = {0}; // 前缀和
        pos[0] = 0;
        for (size_t i = 1; i < RADIX; ++i)
            pos[i] = pos[i - 1] + count[i - 1];

        for (size_t i = 0; i < n; ++i) {
            uint8_t key = (a[i] >> shift) & 0xFF;
            buf[pos[key]++] = a[i]; // 缓存性能差
        }

        std::swap(a, buf); // 可能有内存拷贝瓶颈
    }
}
```

前缀和满足： $$pos[d]=\sum_{k<d}count[k]$$

因此将每个分桶的元素个数转化为结果中每个分桶的起始坐标。