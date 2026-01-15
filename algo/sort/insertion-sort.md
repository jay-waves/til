## 插入排序

在已经有序的局部集合中 `a[0]~a[i-1]`, 寻找新元素 `a[i]` 的插入位置, 寻找插入位置的过程为: 新插入的元素不断和相邻的元素比较大小. 插入排序具有稳定性.

```cpp
void insertion_sort(int* arr, size_t n) {
    for (size_t i = 1; i < n; ++i) {
        int key = arr[i];
        size_t j = i;

        while (j > 0 && arr[j - 1] > key) {
            arr[j] = arr[j - 1];
            --j;
        }
        arr[j] = key;
    }
}
```

注意这里的写法, 每次交换并非两两互换位置.