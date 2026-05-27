基于 AVL 或者 RB 树. 虽然有 Interval (区间), 但是仍以 `interval.low` 为主按 BST 的方式构建. 每个节点记录其及其子树的最大区间上界 high. 

```c
struct Node{
	Interval interval; // 区间 [low, high]
	int max_end;       // 节点及其子树的最大 high 
	Node* left;
	Node* right;
	int height;       // AVL
}

```

## 查询

查询复杂度大概为 `O(log n + k)`

查询和 `[low0, high0]` 区间相交的元素. 
1. 检查当前节点
2. 对于左子树, 如果 `left->max_end >= low0`, 则递归查询左子树.
3. 对于右子树, 如果 `interval.start <= high0`, 则递归查询右子树

## 插入

## 删除
