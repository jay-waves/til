
![|400](../../attach/binary-search-tree.avif)

1. 节点至多有两个分叉, 称为左子树和右子树.
2. 左子树节点值总小于当前节点值
3. 右子树节点值总大于当前节点值

**节点深度**: 从根节点走到当前节点所经历的节点数.

**树高度**: 最深的节点深度.

**平衡二叉树**: 叶子节点的深度差至多为一. 平衡树有多种实现:
- [red-black-tree](red-black-tree.md)
- [avl-tree](avl-tree.md)
- [b-tree](b-tree.md)

**完全二叉树**: 所有叶节点的深度相同, 内部节点的出度 (子节点数) 相同.

## 接口

推荐二叉树引入计数 `cnt`，插入元素时增加计数器；删除元素时，减少计数器，归零后彻底删除。

```cpp
template <class K> 
requires std::totally_ordered<K>
class tree {
   struct node {
	    K k;
	    size_t cnt; 
	    node* l;
	    node* r;
	
	    node(const K& k) : k(k), cnt(1), l(nullptr), r(nullptr) {}
	};
    typedef node<K> node_t;
public:
    node_t* root;

    tree(const tree&) = delete;
    tree& operator=(const tree&) = delete;

    node_t* find_min() { return find_min(root); }
    node_t* find_max() { return find_max(root); }

    void insert(K k) { root = insert(root, k); }
    void remove(K k) { root = remove(root, k); }
    
private:
    node_t* find_min(node_t* cur);
    node_t* find_max(node_t* cur);
    node_t* insert(node_t*cur, const K& k);
    node_t* remove(node_t*cur, const K& k);
}
```

## 实现

### 遍历二叉树

#### 先序遍历

```cpp
void pre_order(node_t* cur) {
	if (!cur) return;
	// process cur node
	pre_order(cur->l);
	pre_order(cur->r);
}
``` 

#### 中序遍历

```cpp
void in_order(node_t* cur) {
	if (!cur) return;
	in_order(cur->l);
	// process current
	in_order(cur->r);
}
```

#### 后序遍历

```cpp
void post_order(node_t* cur) {
	if (!cur) return;
	post_order(cur->l);
	post_order(cur->r);
	// process current 
}
```

#### 同层遍历

```cpp
using std::queue;

void level_order(node_t* cur) {
	if (!cur)
		return;
		
	queue<node_t*> q; 
	q.push(cur);
	while(!q.empty()) {
		node_t* x = q.front();
		q.pop();
		// process x
		if (x->l) q.push(x->l);
		if (x->r) q.push(x->r);
	}
}
```

### 寻找元素

```cpp
node_t* find_min(node_t* cur) {
	if (cur == nullptr) 
		return nullptr;
	while(cur->l)
		cur = cur->l;
	return cur;
}

node_t* find_max(node_t* cur) {
	if (cur == nullptr)
		return nullptr;
	while(cur->r)
		cur = cur->r;
	return cur;
}
```

#### 区间查找？？

![|500](../../attach/binary-search-tree2.avif)

### 插入元素

```cpp
node_t* insert(node_t*cur, const K& k) {
	if (!cur) return new node_t(k);

	if (k == cur->k) {
		++cur->cnt;
	} else if (k < cur->k) {
		cur->l = insert(cur->l,k);
	} else {
		cur->r = insert(cur->r, k);
	}
	return cur;
}
```

### 删除元素

当一个节点必须被删除时，需要用其*右子树中最小节点*替换被删除节点。

```cpp
node_t* remove(node_t*cur, const K& k) {
	if (!cur) 
		return nullptr;

	if (k < cur->k) 
		cur->l = remove(cur->l, k);
	else if (k > cur->k)
		cur->r = remove(cur->r, k);
	else {
		if (cur->cnt > 1) {
			--cur->cnt; 
			return cur;
		}

		if (!cur->l) {
			node_t* r = cur->r;
			delete cur;
			return r;
		}
		if (!cur->r) {
			node_t* l = cur->l;
			delete cur;
			return l;
		}

		// 在通常情况下，需要用*右子树的最小节点*替换被删除节点
		node_t * succ = find_min(cur->r);

		cur->k = succ->k;
		cur->cnt = succ->cnt;

		succ->cnt = 1; // 彻底删除
		cur->r = remove(cur->r, succ->k); 
	}

	return cur;
}
```
