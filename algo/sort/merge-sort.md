## 归并排序

思想是*分治*。将序列不断细分为短序列，直至仅有一个元素（直接认为有序）或有两个元素（一次比较和交换）。接着将短序列向上两两合并为有序的长序列。

具有排序稳定性，因为在*合并*过程中，所有结果按序填入数组，而没有跳跃。

额外空间复杂度为 `O(n)`，主要消耗为用于辅助的临时存储数组 `temp[]`

```cpp
template <class T, class Comp = std::less<>>
requires std::sortable<T*, Comp>
void merge(span<T> left, span<T> right, span<T> out, Comp& comp = {}) {
  size_t i = 0, j = 0, k = 0;
  while (i < left.size() && j < right.size()) {
    if (comp(right[j], left[i])) {
      out[k++] = std::move(right[j++]);
    } else {
      out[k++] = std::move(left[i++]);
    }
  }

  while (i < left.size()) {
    out[k++] = std::move(left[i++]);
  }

  while( j < right.size()) {
    out[k++] =  std::move(right[j++]);
  }
}

template <class T, class Comp = std::less<>>
requires std::sortable<T*, Comp>
void merge_sort_impl(span<T> a, span<T> tmp, Comp& comp = {}) {
  if (a.size() < 2) return;

  const size_t mid = a.size() / 2;
  auto left = a.subspan(0, mid);
  auto right = a.subspan(mid);
  
  auto tmp_left = tmp.subspan(0, mid);
  auto tmp_right = tmp.subspan(mid);

  merge_sort_impl(left, tmp_left, comp);
  merge_sort_impl(right, tmp_right, comp);
  merge(left, right, tmp, comp);

  // copy back
  for (size_t idx = 0; idx < a.size(); ++idx) {
    a[idx] = std::move(tmp[idx]);
  }
}

template <class T, class Comp = std::less<>>
requires std::sortable<T*, Comp>
void merge_sort(span<T> a, Comp comp = {}) {
  if (a.size() < 2) 
    return;
  
  std::vector<T> temp(a.size());
  merge_sort_impl(a, span<T>(temp), comp);
}
```