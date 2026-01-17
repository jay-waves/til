```cpp
template <class K, class Compare>
struct node {
	K k;
	size_t cnt;
	node* l = nullptr;
	node* r = nullptr;
};
```