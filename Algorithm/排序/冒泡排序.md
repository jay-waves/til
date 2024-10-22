

## 冒泡排序

冒泡排序每次比较相邻的两元素, 每次遍历保证最大元素能够浮动到最高处.

```c
void bubble_sort(void *base, size_t num, size_t size, 
								 int (*cmp)(const void*, const void*))
{
	char *arr = base;
	for (size_t i = 1; i < num; ++i) {
		for (size_t j = 1; j < num - i; ++j) {
			if (cmp(arr + j * size, arr + (j - 1) * size))
				swap(arr + j * size, arr + (j - 1) * size, size);
		}
}
```

