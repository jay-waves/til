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
