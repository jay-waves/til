---
revised: 26-01-23
---


## 快速排序

已知最快排序算法。通过一趟排序，将元素分为两部分，其中一部分的最小元素大于另一部分的最大元素，然后将两部分递归处理下去。
1. 选基准：从数列中取出一个基准数（pivot）。 随机选择基准数可以减少最差时间情况。
2. 分区：将所有比基准数大的数移动到其右侧，其他数移动到基准数左侧。
3. 递归：对左右侧区间重复步骤2，递归至区间大小为 1。

快速排序思想是[分而治之](../algorithm.md)。不具有稳定性，因为选择基准数算法被认为是随机的。空间复杂度为 `O(log n)`，主要消耗为递归调用栈深度。

### lomuto partition

```cpp
template <std::random_access_iterator It, class Comp = std::less<>>
requires std::sortable<It, Comp>
void quick_sort(It first, It last, Comp comp = {}) {
    if (last - first < 2) return;

    It pivot_pos = lomuto_partition(first, last, comp);

    quick_sort(first, pivot_pos, comp);       // [first, pivot_pos)
    quick_sort(pivot_pos + 1, last, comp);    // (pivot_pos, last)
}

template <std::random_access_iterator It, class Comp = std::less<>>
requires std::sortable<It, Comp>
It lomuto_partition(It first, It last, Comp comp = {}) {
    // [first, last)
    auto const& pivot = *(last - 1);

    It i = first; 
    for (It j = first; j != last - 1; ++j) {
        if (comp(*j, pivot)) {          
            std::iter_swap(i, j);       
            ++i;
        }
    }
    std::iter_swap(i, last - 1); // move pivot to middle       
    return i;                           
}
```

### 三向分区

```cpp
template <std::random_access_iterator It, class Comp = std::less<>>
requires std::sortable<It, Comp>
std::pair<It, It> three_way_partition(It first, It last, Comp comp = {}) {
    const auto pivot = *(first + (last - first) / 2);

    It lt = first;   
    It i  = first; 
    It gt = last;    

    while (i < gt) {
        if (comp(*i, pivot)) { // *i < pivot
            std::iter_swap(lt, i);
            ++lt;
            ++i;
        } else if (comp(pivot, *i)) { // *i > pivot
            --gt;
            std::iter_swap(i, gt);
            // 注意这里 i 不自增
        } else { // *i == pivot
            ++i;
        }
    }

    // [lt, gt), *lt = p, *(lt-1) < p, *gt > p, *(gt-1) = p
    return {lt, gt};
}

template <std::random_access_iterator It, class Comp = std::less<>>
requires std::sortable<It, Comp>
void quick_sort_three_way(It first, It last, Comp comp = {}) {
    if (last - first < 2) return;

    auto [lt, gt] = three_way_partition(first, last, comp);
    quick_sort_three_way(first, lt, comp);  // < pivot
    quick_sort_three_way(gt, last, comp);   // > pivot
}
```

因为等于 `pivot` 的部分不再参与后续递归，因此收敛会更快。在有大量重复元素的场景中，速度可以接近 $O(n)$。