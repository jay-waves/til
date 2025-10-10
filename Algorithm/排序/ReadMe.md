
| 排序方法                         | 平均时间复杂度       | 最佳时间复杂度     | 最坏时间复杂度     | 额外空间复杂度 | 稳定性[^1] | 基于 |
| -------------------------------- | -------------------- | ------------------ | ------------------ | -------------- | ---------- | ---- |
| [插入排序](insertion%20sort.md) | $O(n^{2})$           | $O(n)$             | $O(n^{2})$         | $O(1)$         | 稳定       | 插入     |
| [希尔排序](shell%20sort.md)          | $\approx O(n^{1.5})$ | $O(n)$             | $O(n^{2})$         | $O(1)$         | 不稳定     | 插入/分治      |
| [选择排序](selection%20sort.md) | $O(n^2)$             | $O(n^2)$           | $O(n^2)$           | $O(1)$         | 不稳定     |  选择    |
| [堆排序](heap%20sort.md)              | $O(n\cdot \log n)$   | $O(n\cdot \log n)$ | $O(n\cdot \log n)$ | $O(1)$         | 不稳定     |   选择   |
| [冒牌排序](bubble%20sort.md#冒泡排序) | $O(n^2)$             | $O(n)$             | $O(n^2)$           | $O(1)$         | 稳定       | 交换     |
| [快速排序](quick%20sort.md)          | $O(n\cdot \log n)$   | $O(n\cdot \log n)$ | $O(n^2)$           | $O( \log n)$   | 不稳定     |  交换/分治    |
| [归并排序](merge%20sort.md)          | $O(n\cdot \log n)$   | $O(n\cdot \log n)$ | $O(n\cdot \log n)$ | $O(n)$         | 稳定       |   分治   |

[^1]: 排序稳定性指: 排序操作前后, 关键字相同的元素的前后相对位置不变. 多关键字排序时, 保持结果稳定性非常重要. 稳定性仅是约定俗称而不是必然, 糟糕的代码会破坏理论上的稳定性, 反之亦然. 在谈论稳定时, 约定当元素键值相同时不交换, 优先使用有最佳性能的局部算法, 在此基础上才进一步讨论.


## 实现细节约定

```c

// 示例 swap() 接口, 宏实现
#define swap(x, y, size) \
	do { \
		 char __temp[size]; \
		 memcpy(__temp, x, size); \
		 memcpy(x, y, size); \
		 memcpy(y, __temp, size); \
	 } while (0)

/*
	&a == &b 时, 如下写法错误

	void swap(int *a, int *b) {
		*a += *b;
		*b = *a - *b;
		*a -= *b;
	}
*/

// 函数实现:
static void swap(void *a, void *b, size_t size)
{
    char tmp;

    if (a == b)
        return;

    while (size--) {
        tmp = *(char *)a;
        *(char *)a = *(char *)b;
        *(char *)b = tmp;

        a = (char *)a + 1;
        b = (char *)b + 1;
    }
}


// 示例值元素
struct val {
	int k1;
	int k2;
	...
};


// 示例 sort 接口
void xxx_sort(void *base, size_t num, size_t size, 
							int (*cmp) (const void*, const void*);

struct sort_params {
	void *base,
	size_t size,
	size_t num.
	int (*cmp)(const void*, const void*),
};
void xxx_sort(struct sort_params *params);


/* 示例 cmp 接口
 * 当 a > b 时, 若 cmp(a, b) > 0, 排序结果为升序排序.
 * 当 a > b 时, 若 cmp(a, b) < 0, 排序结果为降序排序.
 * 若 a > b && cmp(a, b) > 0, 应有 cmp(b, a) < 0
 * 当 a = b 时, 应有 cmp(a, b) = 0, 默认不换序, 保持排序稳定性.
*/
int cmp(const void *a, const void *b) {
	struct val *_a = (struct val *)a;
	struct val *_b = (struct val *)b;
	return _a->k1 - _b->k1;
}
```