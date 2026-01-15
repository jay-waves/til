## 选择排序

选择排序每轮从乱序集合中选出最小元素, 放入有序集合中, 类似线性的最小堆排序. 选择排序被认为不具备稳定性.

```c
void selection_sort(int* arr, size_t n) {
    for (size_t i = 0; i + 1 < n; ++i) {
        size_t min_idx = i;

        for (size_t j = i + 1; j < n; ++j) {
            if (arr[j] < arr[min_idx]) {
                min_idx = j;
            }
        }

        if (min_idx != i) {
            std::swap(arr[i], arr[min_idx]);
        }
    }
}
```