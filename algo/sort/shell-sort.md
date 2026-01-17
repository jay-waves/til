## 希尔排序

对于 $n=[...,3, 2, 1]$, 将数据按希尔数列 $a_{n}=[.., 8, 4, 2, 1]$ 分割为 $len/a_n$ 个子数列, 每个数列独立进行插入排序. 即按不同的步长 $a_{i}$ 对元素进行[插入排序](insertion-sort.md)

希尔排序在初始时采用较大步长来交换和比较, 更快地消除数组中的逆序对, 相比之下插入排序要像冒泡一样以小步长移动. 当步长逐渐变小时, 数组实际已接近有序状态, 此时插入排序不必频繁交换移动数据, 从而提高了整体效率. 希尔排序的思想是*分治*.

选择不同希尔数列, 在不同情况下有不同性能. 总的来说, 希尔排序的复杂度略好于插入排序 $O(n^{2})$, 但不具备稳定性.

```cpp
template <class T, class Comp = std::less<>>
requires std::sortable<T*, Comp>
void shell_sort(span<T> s, Comp comp = {}) {

	const auto n = s.size();
	if (n < 2) return;
	
	for (auto gap = n/2; gap > 0; gap /= 2) {
		for (auto i = gap; i < n; ++i) {
			auto temp = std::move(s[i]);
			
			auto j = i;
			while(j >= gap && comp(temp, s[j-gap])) {
				s[j] = std::move(s[j-gap]);
				j -= gap;
			}
			
			s[j] = std::move(temp);
		}
	}
}
```

> 注意这里的隐式 `swap()` 写法, 减少了 2/3 的赋值操作.

### 常见希尔序列

设 $n$ 为数组长度, $k\in[1,\ 2,\ ..]$

Donald Shell's, $\lfloor \frac{n}{2^{k}}\rfloor$, 如 `8, 4, 2, 1`

T.P. Hibbard's, $2^{k}-1$, 如 `1, 3, 7, 15, ...`

Knuth's, $\frac{3^{k}-1}{2}$, 如 `1, 4, 13, 40, ...`

Sedge's, 
- 奇数形式: $9\times 4^{i}-9\times 2^{i}+1$, 如 `1, 5, 19, 41, 109, ...`
- 偶数形式: $2^{i+2}\times(2^{i+2}-3)+1$

Pratt's, $2^{i}\times 3^{j}$, 如 `1, 2, 3, 4, 6, 8, 9, 12, ...`