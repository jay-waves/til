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

## 编译优化的基本问题

- 公共子表达式化简 (common subexpression elimination, CSE)
	- 参数访问
	- 结构体字段访问
	- 数组字段访问
- 常量传播 (constant propagation)
- 死代码消除 (dead code elimination)

循环优化:
- 减少循环代码执行长度
- 减少代码体积

### DAG vs. Value Numbering

代码中的表达式 $T$, 一般被表示为 AST 树形结构. 但树形内部存在结构性冗余, 被称为*公共子表达式*, 在对 AST 树变换时将导致复杂度指数膨胀, 不利于代码复用和代码分析.[^1]

为了消除 CSE, 使用*有向无环图 (DAG)* 来表达 $T$. 每个节点是一个子表达式, 用边表达表达式间的依赖关系. 和树形结构相比, DAG 中的节点可以有多个父节点. DAG 的等价文本形式是 *SLP (Straight-Line Program)*, 在汇编中进一步化简为*三地址码*.

SLP 是一个有限的指令序列 $\mathcal{P}=(I_{1},I_{2},\dots,I_{m})$, 其中每个指令 $I_{k}$ 形式为 $v_{k}\leftarrow \phi_{k}(v_{1},v_{2},\dots,v_{n})$. 也就是定义了变量间的依赖关系. 



另一种更有效的方式为 Value Numbering, 通过哈希的方法, 为公共子表达式分配唯一编号, 有效地识别等价表达式. 

### SSA 

DU (Definition-Use) Chains: 变量 X 的定义可见, 如何找到所有对 X 的访问.

UD (Use-Definition) Chains: 变量 X 的某个访问可见, 如何找到所有对 X 的可达定义?

如果允许对变量 X 的重复定义 (赋值), DU 和 UD 关系都会很复杂. 引入 SSA (Static Single Assignment), 以及 PSSA (Partial SSA). 

PSSA 中, 大部分操作使用虚拟寄存器表达, 少部分复杂操作仍保留内存操作. 如果变量 X 的定义来自不同的控制流, 用 $\phi$ 来显式表达. 

## 数据流分析

[数据流分析](../../软件分析/数据流分析.md)

## 别名分析

基于包含, 基于合并

## 过程间分析

考虑函数间的调用关系, 也称为*链接时分析 LinkTime Analysis*. 

## 类型状态分析

状态机形式描述

## Lattice (Order)

指某种*偏序集合* (partially ordered set, poset), 其中任意一对元素有唯一的*上确界* (supremum, least upper bound, join) 和*下确界* (infimum, greatest lower bound, meet). 

表达了一种信息精确性的约束关系, 见 https://en.wikipedia.org/wiki/Lattice_(order).

![](../../../attach/Snipaste_2025-09-18_14-18-49.png)

$x\leq y$ iff $x\wedge y =x$. 

$x\wedge y\leq x$


## 参考

将内核编译为 vmlinux.bc : wllvm https://github.com/travitch/whole-program-llvm

CMU15-745. Optimizing Compilers for Moedern Architectures, [Spring 2019](https://www.cs.cmu.edu/afs/cs/academic/class/15745-s19/www/), [Fall 2025](https://www.cs.cmu.edu/~15745/www/)

[^1]: 这个现象被称为 Expression Swell. 参考 [为什么函数求导后, 表达式长度会指数增加?](https://www.zhihu.com/question/609058716/answer/1954152484693058426)