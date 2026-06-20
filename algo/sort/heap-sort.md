## 堆排序

利用堆的性质: 父节点用于大于子节点. 从堆中取出根元素 (最大元素), 并等待堆重新平衡, 不断重复此过程, 最终得到有序序列. 堆排序中, 将堆顶和堆底互换的操作会破坏稳定性. 

```cpp
/*
 * 使用数组实现最大堆
 * 节点 i 的左子节点位置为 2 * i + 1
 * 节点 i 的右子节点位置为 2 * i + 2
 * 节点 i 的父节点位置为 floor((i-1) / 2)
 */

using std::vector; // in vector
using std::swap;   // in algorithm

template <typename T, typename Compare>
class Heap {
public:
    Heap(vector<T> &data, Compare cmp) : data(data), cmp(cmp) {}

	/*
	* 最大堆调整: 删除堆顶后, 使堆重新平衡. 不能用于建立最大堆.
	*/
    void heapify(size_t start, size_t end) {
        size_t dad = start;
        size_t son = dad * 2 + 1;
        while (son <= end) {
            if (son + 1 <= end && cmp(data[son], data[son + 1]))
                son++;
            if (!cmp(data[dad], data[son])) {
                return;
            } else {
                swap(data[dad], data[son]);
                dad = son;
                son = dad * 2 + 1;
            }
        }
    }

	/*
	 * 原地堆排序
	 */
    void sort() {
        size_t len = data.size();
        // 建堆, 自底向上
        for (size_t i = len / 2 - 1; i < len; i--) {
            heapify(i, len - 1);
        }
        for (size_t i = len - 1; i > 0; i--) {
            swap(data[0], data[i]); // 首尾互换
            heapify(0, i - 1);      // 堆尺寸减一
        }
    }

private:
    std::vector<T> &data;
    Compare cmp;
};

template <typename T, typename Compare>
void heap_sort(vector<T> &arr, Compare cmp) {
    Heap<T, Compare> heap(arr, cmp);
    heap.sort();
}


/*
 * sort in ascending order
 heap_sort(arr, std::less<int>());
 * sort in descending order
 heap_sort(arr, std::greater<int>());
*/
```