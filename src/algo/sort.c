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
