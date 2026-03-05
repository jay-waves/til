### 情景

给定数组 `a[0..n-1]`，求每个长度为 `k` 的滑动窗口的最大值。

### 数据结构

维护单调双端队列：队列从头到尾，单调递减
* 存储下标 `i`
* 如果 `a[i]` 大于队尾，弹出队尾。直到队尾大于 `a[i]` 后，入队 `i`
* 如果队头过期，弹出队头，直到队头在窗口 `k` 内。

总复杂度为 $O(2n)$，优于暴力求解 $O(kn)$。

```cpp
using std::deque;
using std::vector;

vector<int> max_sliding_window(const vector<int>& a, int k) {
	deque<int> dq;
	vector<int> ans;
	
	for (int i = 0; i < a.size(); ++i) {
		while(!dq.empty() && a[dq.back()] <= a[i])
			dq.pop_back();
			
		dq.push_back(i);
		
		if (dq.front() <= i - k) 
			dq.pop_front();
			
		if (i >= k-1)
			ans.push_back(a[dq.front()]);
	}
	
	return ans;
}
```