## 快速排序

已知最快排序算法, 每一趟子排序能否确定一个元素的最终位置.
1. 从数列中取出一个基准数. 随机选择基准数可以减少最差时间情况.
2. 将所有比基准数大的数移动到其右侧, 其他数移动到基准数左侧.
3. 对左右侧区间重复步骤2, 递归至区间大小为1.

快速排序思想是分治. 不具有稳定性, 因为选择基准数算法被认为是随机的. 空间复杂度为 `O(log n)`, 主要消耗为递归调用栈深度.

```c
static void quick_sort(void *base, size_t size, int low, int high, 
										int (*cmp)(const void *, const void *))
{
	char *arr = (char *)base;
    if (low < high) {
        char *pivot = arr + high * size;
        int i = low - 1;
        int j;

        for (j = low; j < high; j++) {
            if (cmp(arr + j * size, pivot) < 0) {
                ++i;
                swap(arr + i * size, arr + j * size, size);
            }
        }
        swap(arr + (i + 1) * size, arr + high * size, size);
        int pi = i + 1;

        quick_sort(base, size, low, pi - 1, cmp);
        quick_sort(base, size, pi + 1, high, cmp);
    }
}

void qsort(void *base, size_t num, size_t size, 
						int (*cmp)(const void *, const void *))
{
    if (num < 2) 
	    return;
    
    quick_sort(base, size, 0, num - 1, cmp);
}

```