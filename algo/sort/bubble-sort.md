
## 冒泡排序

冒泡排序每次比较相邻的两元素, 每次遍历保证最大元素能够浮动到最高处.

```cpp
void bubble_sort(int* arr, size_t n) {
    for (size_t i = 0; i + 1 < n; ++i) {
        for (size_t j = 0; j + 1 < n - i; ++j) {
            if (arr[j] > arr[j + 1]) 
                std::swap(arr[j], arr[j + 1]);
        }
    }
}
```

