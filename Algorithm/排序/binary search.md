## 二分查找

二分查找的时间复杂度为 `O(log n)`, 适用于已排序的数组. 如果数组中包含重复的目标值, 二分查找可能返回任一一个的位置, 不保证相对顺序.

### 实现

一般实现: 注意避免直接 `left+right` 导致的溢出.

```c
int binary_search(int arr[], int size, int target) 
{
    int left = 0;
    int right = size - 1;
    
    while (left <= right) {
        int mid = left + (right - left) / 2; 
        
        if (arr[mid] == target) {
            return mid; 
        }
        
        if (arr[mid] < target) {
            left = mid + 1;  // on the right
        } else {
            right = mid - 1; // on the left
        }
    }
    
    return -1; // not found
}
```

递归实现:

```c
int binary_search(int arr[], int left, int right, int target) 
{
    if (left <= right) {
        int mid = left + (right - left) / 2;
        
        if (arr[mid] == target) {
            return mid;
        }
        
        if (arr[mid] < target) {
            return binary_search(arr, mid + 1, right, target);
        } else {
            return binary_search(arr, left, mid - 1, target);
        }
    }
    
    return -1;
}
```

### 扩展

```c
/* more or equal searc, >= */
int binary_ge_search(const int arr[], int size, int target)
{
	int left = 0;
	int right = size - 1;

	while (left <= right ) {
		int mid = left + (right - left) / 2; 

		if (arr[mid] >= target)
			right = mid - 1;
		else 
			left = mid + 1;
	}

	if (low == size + 1)
		return -1;
	return left;
}

/* more search, > */
int binary_gt_search(const int arr[], int size, int target)
{
	int left = 0;
	int right = size - 1;

	while (left <= right ) {
		int mid = left + (right - left) / 2; 

		if (arr[mid] > target)
			right = mid - 1;
		else 
			left = mid + 1;
	}

	if (low == size + 1)
		return -1;
	return left;
}
```