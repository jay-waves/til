C++17 引入的 Parallel STL ，适合计算密集的并行操作，不适合共享状态（UB）

```cpp
#include <execution>

std::execution::seq; // 串行
std::execution::par; // 多线程并行
std::execution::par_unseq; // 并行，并允许向量化重排
std::execution::unseq;  // 仅向量化
```

### 算法支持 

```cpp 
#include <algorithm>

std::sort(std::execution::par, v.begin(), v.end());
for_each(...); // 注意不要共享状态
transform(...);
reduce(...);
find(...);
copy(...);
count(...);
```