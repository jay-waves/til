头文件 `<algorithm>` 

### 排序

| 名称               | 用途          | `const` | 头文件   |
| ------------------ | ------------- | ------- | -------- |
| `accumulate`       | 元素累计      | 是      | numeric  |
| `binary_search`    | 二分查找      | 是      |          |
| `copy`             | 复制          | 否      | algobase |
| `copy_backward`    | 逆向复制      |         | algobase |
| `count`            | 计数          |         | algo     |
| `count_if`         | 特定i套件计数 |         | algo     |
| `fill`             | 改填元素值    |         | algobase |
| `find`             | 顺序查找      |         | algo     |
| `find_end`         |               |         |          |
| `find_first_of`    |               |         |          |
| `for_each`         |               |         |          |
| `max`              |               |         | algobase |
| `min`              |               |         | algobase |
| `merge`            |               |         | algo     |
| `partial_sort`     |               |         | algo     |
| `power`            |               |         |          |
| `random_shuffle`   |               |         |          |
| `random_sample`    |               |         |          |
| `remove`           |               |         |          |
| `remove_if`        |               |         |          |
| `replace`          |               |         |          |
| `reverse`          |               |         |          |
| `rotate`           |               |         |          |
| `search`           |               |         |          |
| `set_difference`   |               |         |          |
| `set_intersection` |               |         |          |
| `set_union`        |               |         |          |
| `sort`             |               |         |          |
| `stable_sort`      |               |         |          |
| `swap`             |               |         |          |
| `transform`        |               |         |          |
| `unique`           |               |         |          |

### sort 

```cpp
// 从大到小排序
sort(v.begin(), v.end(), [](int a, int b){
	return a > b;
});
```

自定义比较器时，需要回答：a 是否应该排序在 b 前面。同时，必须满足自反性 `comp(a, a) == false` （没理解为啥）

### stable_sort 

`stable_sort` 默认是归并排序，因此需要额外内存。

### unique_copy 

```cpp
list<Entry> res, vec;
sort(vec.begin(), vec.end());
unique_copy(vec.begin(), vec.end(), back_inserter(res));
```

### find

```cpp 
// does s contain c?
find(s.begin(), s.end(), c) != s.end()

// find all occurrences 
for (auto p : find_all(s, 'a'))
	...
```

实现原理:

```cpp
template<typename T>
using Iterator = typename T::iterator; 

template<typename C, typename V>
vector<Iterator<C>> find_all(C& c, V v) {
	vector<Iterator<C>> res;
	for (auto p = c.begin(); p != c.end(); ++p)
		if (*p == v) res.push_back(p);
	return res;
}
```

#### find_if

需要传入一个*函数子*.

```cpp
find_if(m.begin(), m.end(), GreaterThan{42}); // map<string, int>& m
struct GreaterThan {
	int val;
	GreaterTahn(int v) : val{v} {}
	bool operator() (const pair<string, int>& r) { return r.second > val; }
}

// or
auto p = find_if(m.begin(), m.end(), [](const pair<string, int>& r) { return r.second > 42; });
```

#### bound

`std::lower_bound(arr, trg)` 要求 `arr` 是非降序的，返回第一个 `arr[i]>=trg` 的坐标 `i`。

`std::upper_bound(arr, trg)` 要求 `arr` 是非降序的，返回第一个 `arr[i]>trg` 的坐标 `i`。
