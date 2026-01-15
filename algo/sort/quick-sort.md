## 快速排序

已知最快排序算法. 通过一趟排序, 将元素分为两部分, 其中一部分的最小元素大于另一部分的最大元素, 然后将两部分递归处理下去.
1. 选基准: 从数列中取出一个基准数 (pivot). 随机选择基准数可以减少最差时间情况.
2. 分区: 将所有比基准数大的数移动到其右侧, 其他数移动到基准数左侧.
3. 递归: 对左右侧区间重复步骤2, 递归至区间大小为1.

快速排序思想是[分而治之](../algo.md). 不具有稳定性, 因为选择基准数算法被认为是随机的. 空间复杂度为 `O(log n)`, 主要消耗为递归调用栈深度.

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

### hoare partition 

Hoare 分区方法, 优势是交换次数少.

```cpp

// Hoare partition scheme
// - Range: [first, last)
// - Returns: iterator p such that every element in [first, p] is <= pivot
//            and every element in (p, last) is >= pivot (w.r.t. comp)
// - Note: pivot is not guaranteed to end up at its final sorted position.
template <std::random_access_iterator It, class Comp = std::less<>>
requires std::sortable<It, Comp>
It hoare_partition(It first, It last, Comp comp = {}) {
    // Choose pivot (common simple choice: middle element)
    const auto pivot = *(first + (last - first) / 2);

    It i = first - 1;
    It j = last;

    while (true) {
        // move i right until *i is NOT < pivot
        do { ++i; } while (comp(*i, pivot));

        // move j left until *j is NOT > pivot  <=> pivot is NOT < *j
        do { --j; } while (comp(pivot, *j));

        if (i >= j) return j;          // split point
        std::iter_swap(i, j);
    }
}

template <std::random_access_iterator It, class Comp = std::less<>>
requires std::sortable<It, Comp>
void quick_sort_hoare(It first, It last, Comp comp = {}) {
    if (last - first < 2) return;
    It p = hoare_partition(first, last, comp);
    quick_sort_hoare(first, p + 1, comp); // [first, p+1)
    quick_sort_hoare(p + 1, last, comp);  // [p+1, last)
}
```