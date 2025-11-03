BMC (Bounded Model Checking) 是一种形式化验证技术. 通过限制循环等控制流深度, 将模型检查问题约束为可满足性问题 (SAT).

对于给定系统 $M$ 和属性断言 $P$, BMC 展开 $M$ 的状态迁移关系 $k$ 次, 得到一个描述长度为 $k$ 的执行路径的逻辑公式, 同时在其内编码断言 $P$. 如果上述公式可满足, SAT 求解器返回一个实际的违例路径, 表示 $k$ 步以内存在属性违例. 如果公式不可满足, 则逐步增大 $k$ 来扩大整个状态空间的搜索范围.

@clarke2001

BMC 的主要缺陷: 随着 $k$ 增加, 状态爆炸. 不具备完备性, 只能找到违例. 

## CBMC 

CBMC (@clarke2004) 是第一个可分析 ANSI-C 全集的 BMC 框架.

大概步骤如下:
1. Preprocessed 
2. loop are unwound: `while`, `goto`. 展开到 $n$ 步.
3. (recursive) function calls are expanded: 将函数内联, 将递归限制在 $n$ 步.
4. Transform into SSA form (requiring a [pointer analysis](pointer%20analysis.md))

![](/attach/cbmc.avif)

此时程序被形式化为 $C,P$, 接着用 SAT 求解器, 求解合取范式 (CNF): ${} C\wedge \lnot P {}$.
- $C$: constraints. 约束, 指程序本身的逻辑约束
- $P$: property. 性质, 如安全条件和用户规定的断言.

## IC3 

@een2011


## 参考

另一种常见的 Model Checking 方法是 BDD.