### less

`std::less` 是函数对象, 用于执行 `<`

```cpp
// in <functional>

template <class T = void>
struct less {
	constexpr bool operator()(const T& lhs, const T& rhs) const {
		return lhs < rhs;
	}
};
```

### greater

`std::greater` 是函数对象, 用于执行 `>`

```cpp
// in <functional>

template <class T = void>
struct less {
	constexpr bool operator()(const T& lhs, const T& rhs) const {
		return lhs > rhs;
	}
};
```