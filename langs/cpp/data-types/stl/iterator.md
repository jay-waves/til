
使用默认迭代器时, 只需定义 `begin(), end()`, 就可以使用 C++ range-for 循环语法.
```cpp
template<typename T>
class Vector {
public:
	T* begin() { return size_ ? &data_[0] : nullptr; }
	T* end() { return begin() + size_; }
private:
	size_t size_;
	T* data_;
};


for (auto& v : my_vecs)
	...
```

复杂类型需要自定义 *标准迭代器*, 其有四个核心要求:
- 支持拷贝构造 (Copy Constructible). 
- 支持拷贝赋值 (Copy Assignable)
- 支持析构, 即支持 RAII, 自动释放资源 (Destructible)
- 支持交换 (Swappable). 

修改容器结构时, 大部分容器的迭代器将失效. 基于链式结构的容器的迭代器更加稳定, 而内存需要重新分配的容器迭代器容易失效, 比如 `vector.resize()` 或 `unordered_map.rehash()` 会导致所有迭代器失效.

## back_inserter 

用于将元素插入到容器末尾, 而不是覆盖原始元素. 实际是不断调用 `push_back()`

```cpp
copy(vec.begin(), vec.end(), std::back_inserter(vec1));

transform(nums.begin(), nums.end(), back_inserter(squares), 
		[](int n) { return n * n; } );
```

## 分类

STL 将算法和容器独立设计, 然后通过迭代器将两者联系在一起. 在 c++20 中, 下述迭代器能力均被定义为 concepts.

迭代器能力:
* range-for 迭代能力. 所有迭代器均支持.
* 前向迭代: `++`
* 读写: `*it`, 返回 `T&`
* 相等比较: `==, !=`. 在多次迭代中, `it` 指向的位置是一致的. 如果插入或消耗元素后, 所有旧迭代器失效, 那么该容器就不具备该能力. 标准库的平衡树有多次遍历能力.
* 后向迭代: `--it`
* 随机访问 (random access): `it[n], it-n, it-n, it1 - it2, it1 < it2, <=, >, >=`  迭代器算术比较, 能以跳跃的方式推进迭代器.
* 连续存储 (contiguous): 支持 `(addressof(*it) + n)` 计算地址
* `const`: 支持 `it begin() const` 和 `it end() const`.


| 能力\迭代器                | 基础 | 输出 | 输入 | 前向 | 双向 | 随机存取 | 连续存储 |
| -------------------------- | ---- | ---- | ---- | ---- | ---- | -------- | -------- |
| 前向迭代, 拷贝, 赋值, 析构 | 1    | 1    | 1    | 1    | 1    | 1        | 1        |
| `*it` 读                   |      |      | 1    | 1    | 1    | 1        | 1         |
| `*it` 写                   |      | 1    |      | 1    | 1    | 1        | 1        |
| 相等比较 (多次遍历)                   |      |      |      | 1    | 1    | 1        | 1        |
| 后向迭代                   |      |      |      |      | 1    | 1        | 1        |
| 随机访问                   |      |      |      |      |      | 1        | 1        |
| 连续存储                   |      |      |      |      |      |          | 1        |


```cpp
for_each(iterator beg, iterator end, _callback); // 返回一个函数对象.
/*

template<class _InIt,class _Fn1> inline
void for_each(_InIt _First, _InIt _Last, _Fn1 _Func)
{
	for (; _First != _Last; ++_First)
		_Func(*_First);
}

*/

transform(iterator beg1, iterator end1, iterator beg2, _callback);  // 将指定容器区间搬运到另一个容器. 不会自动分配目标容器内存.
```

STL 容器提供的迭代器能力:
* `vector::iterator, array::iterator` ------> 连续迭代器
* `deque::iterator` -------> 随机访问迭代器
* `list::iterator` -------> 双向迭代器

## 自定义迭代器

### 仅支持 range-for 遍历

```cpp 

class vec {
public:
	struct iterator {
		T* p;
		T& operator*() const { return *p }
		iterator& operator++() { ++p; return *this; }
		bool operator!=(const iterator& other) const { return p != other.p; }
	};
	
	iterator begin() { return iterator{data_}; }
	iterator end() { return iterator{data_ + size_}; }

private:
	T* data_;
	std::size_t size_; 
};

```

如果容器内部是连续的, 那么连定义 iterator 都不需要:

```cpp
struct buf {
    T* data;
    std::size_t size;

    T* begin()       { return data; }
    T* end()         { return data + size; }

    const T* begin() const { return data; }
    const T* end()   const { return data + size; }
};
```

### 支持 `<algorithm>`

直接复用 vector:
```cpp
class vec {
public:
	using value_type = T;
	using iterator = std::vector<T>::iterator;
	using const_iterator = std::vector<T>::const_iterator;
	
	iterator begin() { return data_.begin(); }
	iterator end() { return data_.end(); }
	const_iterator begin() const { return data_.begin(); }
	const_iterator end() const { return data_.end(); }
	
private:
	std::vector<T> data_;
};
```

自定义连续迭代器:

```cpp
class vec {
public:
	using value_type = T;
	using iterator = T*;
	using const_iterator = const T*;
	
	iterator begin() { return data_; }
	iterator end() { return data_ + size_; }
	...
	
private:
	T* data_;
	std::size_t size_;
};

```

## ranges 

C++20 Ranges, 由 ranges-v3 迭代而来. 优点是更函数式, 缺点是概念多, 编译慢.

如果容器支持 `begin(r)/end(r)`, 那么它就满足 ranges 协议 (概念). 在此之上, ranges 实际能力由容器的 iterator 等级决定, 比如 forward_iterator 意味着 forward_range.

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