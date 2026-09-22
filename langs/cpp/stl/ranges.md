## Ranges 

C++20 `<ranges>`, 可以直接与所有 STL 算法/容器 互操作. `ranges` 支持构造一个**延迟计算的流水线**, 当且仅当对其遍历时, 才实际计算. 其生命周期依赖于原数据视图. 

**尽量简洁地使用, `<ranges>` 报错信息复杂, 很难看懂和调试.**  

```cpp
#include <ranges>
#include <algorithm> // for std::ranges::for_each 

{
	std::vector<int> vec = {1, 2, 3, 4, 5, 6, 7, 8, 9};

	auto rng = vec
			| std::views::filter([](int x){ return x % 2 == 0; })
			| std::views::transform([](int x){ return x * 2; })
			| std::views::take(2);  

	for ( int v: rng)
		..... // rng --> 4, 8
}
```

| views       |                       |
| ----------- | --------------------- |
| `filter`    | 过滤                  |
| `transform` | 映射                  |
| `take`      | 取前 N 个             |
| `drop`      | 跳过前 N 个           |
| `reverse`   | 反转                  |
| `join`      | 拼接 (二维拼接为一维) |
| `split`     | 拆分                  |
| `chunk_by`  |                       |
| `slide`     |                       |
| `adjacent`  |                       |
| `zip`       |                       |
| `enumerate` |                       |
| `keys`      | first elements of pair-lie values                       |
| `values`            |  seconde elements of pair-like value                      |


注意, 如果没有将结果拷贝一份 (存入容器), 那么每次遍历一个视图时, 都会重新计算一遍.

```cpp 
// 手动收集计算结果

std::vector<int> result (
	rng.begin(), r.end()
);

// 或者, 
std::vector<int> result;
std::ranges::copy(rng, std::back_inserter(result));

// 或者, C++23 新增 std::ranges::to
auto rng = v | .... | std::ranges::to<std::vector<int>>();

```

## Ranges 协议

如果容器支持 `begin(r)/end(r)`, 那么它就满足 ranges 协议 (概念). 在此之上, ranges 实际能力由容器的 [iterator](iterator.md) 等级决定, 比如 forward_iterator 意味着 forward_range.

`view` 是轻量无拥有权的 ranges 协议, 比如 `string_view, span` 就支持 `view` 协议.

通过迭代器支持 `ranges`:

```cpp
struct vec {
	struct iterator {
		// 支持 *, ++, ==
	};
	iterator begin();
	iterator end();
}
```

裸 `ranges`:

```cpp
struct vec {

	T* begin(); // begin/end 返回同类型, 满足 common_range 概念
	T* end();
	size_t size() const; // 满足 sized_range 概念
	
	T* data_; // 满足 ranges::contiguous_range 概念
	size_t size_;
}
```


## 几种惯用法

展开多层容器：

```cpp
vector<vector<int>> groups;

for (int x : groups | std::views::join) {}
```

遍历 KV 时，比结构化绑定简洁：

```cpp
for (const auto& k : table | std::views::keys) {}

for (const auto& v : table | std::views::values) {}
```

生成序列:

```cpp
for (int i : std::views::iota(0, 100)) {} 
// 0 ... 99
```

查找、判断和聚合时：

```cpp
bool has_pending = std::ranges::any_of(tasks, 
		[](const Task& t){ return !t.done; });
```