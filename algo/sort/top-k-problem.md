## 最大的 K 个元素

### 排序法

数组整体排序, 然后选出前 $k$ 个元素. 复杂度为 $O(n\log n)$

### 局部排序

只选出前 $k$ 个数进行(冒泡)排序, 结果有序. 复杂度为 $O(nk)$

### 堆

只选出前 $k$ 个数, 但是无序. 堆的规模维持在 $k$ 个元素. 以最小堆为例, 新元素比堆顶大, 那么就弹出堆顶然后压入新元素, 最终选出 $k$ 个最大的元素. 

复杂度为 $O(n\log k)$

### 分治法

1. 找到第 $k$ 大的元素
2. 做一次[分区](quick-sort.md), 此后左半区就是前 $k$ 个数.

```
int random_select(arr, low, high, k) {
	if(low == high) return arr[low];
	
	i = partition(arr, low, high);
	temp = i - low;
	if (temp >= k)
		return random_select(arr, low, i-1, k)
	else 
		return random_select(arr, i+1, high, k-i)
}
```

## 第 K 大的元素