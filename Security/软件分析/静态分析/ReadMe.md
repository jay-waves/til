数据流分析: 
- 常量, 范围传播 --> 基于格的
- 别名/指针分析
- 污点分析
- 后向数据流分析

约束求解 (SMT) 及符号执行

类型分析

控制流分析:
- CFG 
- 可达性分析
- 支配性分析

粒度:
- 上下文敏感 (context)
- 流敏感 (flow-sensitvie): 考虑语句执行顺序
- 路径敏感 (path): 区分分支条件 (一般配合 SMT)
- 字段敏感: 细化结构体字段
- 线程敏感: 并发语义, 发生序关系, 锁集

## 现有框架

- LLVM Builtins: `mem2reg, scalar-evolution, aa`
- [SVF](https://svf-tools.github.io/SVF/): 别名分析, 流分析
- 



## Lattice (Order)

指某种*偏序集合* (partially ordered set, poset), 其中任意一对元素有唯一的*上确界* (supremum, least upper bound, join) 和*下确界* (infimum, greatest lower bound, meet). 

格 $(L,\leq)$ 的任意元素对 $\set{a, b}\subseteq L$, 皆有一个上确界 $a\vee b$ 和下确界 $a\wedge b$. 

抽象来讲, 偏序表示了一种信息包含关系, $a\leq b$ 表示 $a$ 的信息更加精确. 上确界表示两个状态的最小上界, 表示合并信息; 下确界表示两个状态的最大下界, 表示交叉约束.

### 子集关系

子集关系是一种偏序: $U=\set{1, 2, 3}$, 它的子集 $\set{1}\leq \set{1, 2}$; 但是 $\set{1,2}$ 和 $\set{2,3}$ 不可比, 因为无子集关系. 

### 区间包含关系

$[a_{1},b_{1}]\leq [a_{2},b_{2}]$ 等价于 $a_{2}\leq a_{1}$ 且 $b_{1}\leq b_{2}$.

## 参考

将内核编译为 vmlinux.bc : wllvm https://github.com/travitch/whole-program-llvm