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

#define get(base, size, index) \
	((void *)((char *)(base) + (index) * (size)))
#define copy(dest, src, size) \
	memcpy((dest), (src), (size))

void shell_sort(void *base, size_t num, size_t size,
								int (*cmp)(const void*, const void*))
{
	char *arr = base, temp[size];
	size_t i, j, gap = num;
	while (gap > 1) {
		gap = gap / 2;
		for (i = gap; i < num; ++i) {
			memcpy(temp, arr + i * size, size);
			for (j = i; (j >= gap); j -= gap) {
				if (cmp(arr + (j - gap) * size, temp) <= 0)
					 break;
				memcpy(arr + j * size, arr + (j-gap) * size, size);
			}
			memcpy(arr + j * size, temp, size);
		}
	}
}
