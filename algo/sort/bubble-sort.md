
## 冒泡排序

冒泡排序每次比较相邻的两元素, 每次遍历保证最大元素能够浮动到最高处.

```cpp
template <std::random_access_iterator It, typename Comp = std::less<>>
requires std::sortable<It, Comp>
void bubble_sort(It first, It last, Comp cmp = {}) {
  if (first == last) return;

  for (It i = last; i != first; --i) {
    for (It j = first + 1; j != i; ++ j) {
      if (cmp(*j, *(j-1))) 
	      std::iter_swap(j, j-1);
    }
  }
}
```



