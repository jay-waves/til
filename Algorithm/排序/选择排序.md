## 选择排序

选择排序每轮从乱序集合中选出最小元素, 放入有序集合中, 类似线性的最小堆排序. 选择排序通常被认为不具备稳定性.

```c
void selection_sort(void *base, size_t num, size_t size, 
										int (*cmp)(const void *, const void *)) 
{
    char *arr = (char *)base;
    for (size_t i = 0; i < num - 1; i++) {
        size_t min_idx = i;
        for (size_t j = i + 1; j < num; j++)
            if (!cmp(arr + j * size, arr + min_idx * size))
                min_idx = j;
        if (min_idx != i)
            swap(arr + i * size, arr + min_idx * size, size);
    }
}
```