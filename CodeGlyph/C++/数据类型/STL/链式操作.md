## Ranges 

C++20 `<ranges>`, 可以直接与所有 STL 算法/容器 互操作. `ranges` 支持构造一个**延迟计算的流水线**, 当且仅当对其遍历时, 才实际计算. 其生命周期依赖于原数据视图. 

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