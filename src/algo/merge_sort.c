struct msort_params {
	void *base;  // arr
	size_t size; // size of arr[0]
	size_t num;  // sizeof(arr)/sizeof(num)
	int (*cmp)(const void*, const void*);
};

# 使用辅助数组 temp[], 也叫双路归并排序
static void merge(struct msort_params *params, void *temp,
							int left, int leftend, int rightend)
{
    char *arr = (char *)params->base;
    char *t = (char *)temp;
    size_t size = params->size;
    int (*cmp)(const void*, const void*) = params->cmp;
    
    int i = left;
    int j = leftend + 1;
    int q = left;

    while (i <= leftend && j <= rightend) {
        if (cmp(arr + i * size, arr + j * size) <= 0) {
            memcpy(t + q * size, arr + i * size, size);
            ++q; ++i;
        } else {
            memcpy(t + q * size, arr + j * size, size);
            ++q; ++j;
        }
    }
    while (i <= leftend) {
        memcpy(t + q * size, arr + i * size, size);
        ++q; ++i;
    }
    while (j <= rightend) {
        memcpy(t + q * size, arr + j * size, size);
        ++q; ++j;
    }
    for (i = left; i <= rightend; i++) {
        memcpy(arr + i * size, t + i * size, size); // copy back
    }
}

static void merge_sort(struct msort_params *params, void *temp, 
											int left, int right) 
{
    int center;
    if (left < right) {
        center = (left + right) / 2;
        merge_sort(params, temp, left, center);
        merge_sort(params, temp, center + 1, right);
        merge(params, temp, left, center, right);
    }
}

int msort(struct msort_params *params)
{
	size_t num = params->num;
	size_t size = params->size;
	if (num < 2) return 0;
		
	void *temp = malloc(size * num);
	if (!temp) return -ENOMEM

	merge_sort(params, temp, 0, num - 1);
	return 0;
}
