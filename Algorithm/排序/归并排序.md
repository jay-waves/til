## 归并排序

思想是*分治*, 将序列不断细分为短序列, 直至仅有一个元素 (直接认为有序) 或有两个元素 (一次比较和交换). 接着将短序列向上两两合并为有序的长序列. 

具有排序稳定性, 因为在*合并*过程中, 总会将左侧子部分先放入结果数组中 (先序序列). 

额外空间复杂度为 `O(n)`, 主要消耗为用于辅助的临时存储数组 `temp[]`


```c
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
```

> 为什么要外部传入 `tmp[]` ?
>
> 避免频繁分配内存. 用 `static` 声明 `tmp` 也不能解决问题, 无法动态声明大小.

### C++实现

> PS: 拿纯 C 写泛型, 太他妈痛苦了.

```cpp
template<typename T, typename Compare = std::less<T>>
class MergeSorter {
 public:
  MergeSorter(std::vector<T>& data, Compare comp = Compare()) 
      : data_(data), comp_(comp) {}

  void Sort() {
    if (data_.size() < 2) return;
    std::vector<T> temp(data_.size());
    MergeSort(0, data_.size() - 1, temp);
  }

 private:
  void MergeSort(int left, int right, std::vector<T>& temp) {
    if (left < right) {
      int center = (left + right) / 2;
      MergeSort(left, center, temp);
      MergeSort(center + 1, right, temp);
      Merge(left, center, right, temp);
    }
  }

  void Merge(int left, int left_end, int right_end, std::vector<T>& temp) {
    int i = left;
    int j = left_end + 1;
    int q = left;

    while (i <= left_end && j <= right_end) {
      if (comp_(data_[i], data_[j])) {
        temp[q++] = data_[i++];
      } else {
        temp[q++] = data_[j++];
      }
    }
    while (i <= left_end) {
      temp[q++] = data_[i++];
    }
    while (j <= right_end) {
      temp[q++] = data_[j++];
    }
    for (i = left; i <= right_end; ++i) {
      data_[i] = temp[i];  // Copy back
    }
  }

  std::vector<T>& data_;
  Compare comp_;
};

/*
using like:
std::vector<int> arr = {...};
MergeSorter<int, std::greater<int>> sorter_desc(arr);
sorter_desc.Sort();
*/
```