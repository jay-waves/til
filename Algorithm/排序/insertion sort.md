## 插入排序

在已经有序的局部集合中 `a[0]~a[i-1]`, 寻找新元素 `a[i]` 的插入位置, 寻找插入位置的过程为: 新插入的元素不断和相邻的元素比较大小. 插入排序具有稳定性.

```c
#define swap(arr, idx1, idx2, size) \
	do { \
		 char __temp[size]; \
		 memcpy(__temp, arr + idx1 * size, size); \
		 memcpy(arr + idx1 * size, arr + idx2 * size, size); \
		 memcpy(arr + idx2 * size, __temp, size); \
	 } while (0)
	 
void insertion_sort(void *base, size_t num, size_t size, 
									int (*cmp)(const void*, const void*))
{
	char *arr = (char *)base;
	for (size_t i = 1; i < num; ++i) {
		for (size_t j = i; j > 0; --j) {
			if (!cmp(arr + (j - 1) * size, arr + j * size))
				break;
			swap(arr , j - 1, j, size);
		}
	}
}
```