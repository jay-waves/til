## 二分查找

二分查找的时间复杂度为 `O(log n)`, 适用于已排序的数组. 如果数组中包含重复的目标值, 二分查找可能返回任一一个的位置, 不保证相对顺序.

### 实现

一般实现: 注意避免直接 `left+right` 导致的溢出.

```c
int binary_search(int arr[], int size, int target) 
{
    int left = 0;
    int right = size; // [left, right)
    
    while (left < right) {
        int mid = left + (right - left) / 2; 
        
        if (arr[mid] == target) {
            return mid; 
        }
        
        if (arr[mid] < target) {
            left = mid + 1;  // on the right
        } else {
            right = mid; // on the left
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

### first great or equal search 

`std::lower_bound(arr, trg)` 要求 `arr` 是非降序的，返回第一个 `arr[i]>=trg` 的坐标 `i`。

`std::upper_bound(arr, trg)` 要求 `arr` 是非降序的，返回第一个 `arr[i]>trg` 的坐标 `i`。

```cpp
int ge_search(const vector<int>& arr, int target) {
	auto it = std::lower_bound(arr.begin(), arr.end(), target);
	if (it == arr.end())
		return -1;
	return static_cast<int>(it - arr.begin());
}

int first_ge(const vector<int>& arr, int trg) {
	int l = 0, r = arr.size();
	
	while (l < r) {
		int m = l + (r - l) / 2;
		
		if (arr[m] >= trg)
			r = m;
		else 
			l = m + 1;
	}
	
	if (l == arr.size())
		return -1;
	return l; // m 不能保证稳定，必须返回 l 或 r
}

int first_gt(const vector<int>& arr, int trg) {
	int l = 0, r = arr.size();
	
	while (l < r) {
		int m = l + (r - l) / 2;
		
		if (arr[m] > trg)
			r = m;
		else 
			l = m + 1;
	}
	
	if (l == arr.size())
		return -1;
	return l;

}
```

### last lower or equal than search

`lt` 等价于 `ge - 1`

```cpp
int last_lt(const vector<int>& a, int trg) {
	int l = 0, r = a.size(); // [l, r)
	
	while (l < r) {
		int m = l + (r - l) / 2;
		if (a[m] >= trg)
			r = m;
		else 
			l = m + 1;
	}
	
	if (l == 0)
		return -1;
	return l - 1;
}
```