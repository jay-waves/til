#import "../../../appx/theme.typ" : tufte, note
#show: tufte

#let vec(x) = $upright(bold(#x))$

= 最小二乘解

线性方程组 $A upright(bold(x)) = upright(bold(b))$ 无解时, 最小二乘法用于找到最小化误差平方和的近似解.

#import "@preview/cetz:0.5.2"

#cetz.canvas(length: 13mm, {
  import cetz.draw: *

  let o = (0, 0)
  let p = (3.8, 1.1)
  let b = (3.8, 4.2)

  line(
    o, (2.8, -1), (6.5, 1), (3.5, 2),
    close: true,
    fill: luma(245),
    stroke: gray + 0.7pt,
  )

  // 投影
  line(o, p, mark: (end: ">"), stroke: gray + 1pt)
  content((2.1, 0.35), [$A hat(bold(x))$])

  // 向量 b
  line(o, b, mark: (end: ">"), stroke: blue + 1.3pt)
  content((4, 4.25), anchor: "west", text(fill: blue)[$bold(b)$])

  // 残差
  line(
    p, b,
    stroke: (paint: black, thickness: 0.7pt, dash: "dashed"),
  )
  content((4, 2.7), anchor: "west", [$bold(r)$])

  // 直角标记
  line(
    p, (4.1, 1.1), (4.1, 1.4), (3.8, 1.4),
    stroke: black + 0.6pt,
  )

  content((3.7, -0.45), [$upright("Col")(A)$])
})

将图中平面视为矩阵 $A$ 的列空间, 蓝色向量为 $vec(b)$. 如果 $vec(b)$ 位于 $A$ 列空间之中, 非齐次线性方程组有解; 如果 $vec(b)$ 不位于 $A$ 列空间之中, 其近似解为 $vec(b)$ 在 $A$ 列空间中的投影. 问题等价于求 $A$ 列空间向量 $A vec(x)$ (基底为 $A$, 坐标为 $vec(x)$), 使两向量距离最小: 

$ upright("min") norm( A vec(x) - vec(b))^2 $

$A vec(x)$ 为 $vec(b)$ 投影时, 向量 $A vec(x) - vec(b)$ 和 $A ( vec(a)_1, . . . vec(a)_n )$ 列空间垂直: 

$
forall vec(a)_i in A, med ( A vec(x) - vec(b), med vec(a)_i ) = 0
$

等价于: $ A^T ( vec(b) - A vec(x) ) = 0 $

解得: $ vec(x) = ( A^T A )^(- 1) A^T vec(b) $

== 另一问题背景

线性回归中, 为了找到数据点 $( x_1 , y_1 ) , ( x_2 , y_2 ) , dots.h , ( x_n , y_n )$ 间的关系, 我们建立线性模型: 

$ y_i = a_1 x_(i 1) + a_2 x_(i 2) + dots.h + a_m x_(i m) + epsilon.alt_i $

其中 $a_1, a_2, dots.h, a_m$ 是要求解的参数, $epsilon.alt_i$ 是误差项. 上式用矩阵表示为:


$ vec(y) = X vec(a) + vec((epsilon.alt)) $


其中 $x_(i j) in X_(n times m)$ 指第 $i$ 个数据点的第 $j$ 个特征.

最小二乘法的目标是找到参数向量 $vec(a)$, 使得模型预测值与实际观测值之间的差距 (误差平方和) 最小化. 误差平方和表示为: 

$ S ( vec(a) ) = sum_(i = 1)^n epsilon.alt_i^2 = sum_(i = 1)^n ( y_i - f ( x_i ) )^2 $

等价于:

$ S ( vec(a) ) = ( vec(y) - X vec(a) )^T ( vec(y) - X vec(a) ) $


对 $S ( vec((a)) )$ 求导, 找到其极小值:


$ frac(partial, partial vec(a)) S ( vec(a) ) = - 2 X^T ( vec(y) - X vec(a) ) = 0 $


得到最小二乘解 (闭式解):

$ vec(a) = ( X^T X )^(- 1) X^T vec(y) $

= Moore-Penrose PseudoInverse 

有 $A in RR^(m times n)$ ，定义其伪逆为 $A^dagger$ ，唯一满足以下四个条件：

$
  A A^dagger A &= A,\
  A^dagger A A^dagger &= A^dagger, \
  (A A^dagger)^top &= A A^dagger, \
  (A^dagger A)^top &= A^dagger A
$

== 不同情形下的伪逆形式

如果 $A$ 是可逆方阵，那么 $A^dagger = A^(-1)$

=== 高瘦矩阵（列满秩）

当 $"rank"(A) = n,quad m eq.gt n$，即，各列线性无关，$A^top A$ 可逆：

$ A^dagger = (A^top A)^(-1) A^top $

此时 $A^dagger$ 称为 $A$ 的左逆：

$ A^dagger A = I_n $

#linebreak()

此时线性系统是过定的，通常没有精确解，可用于求*唯一的最小二乘解*：

$ x^* = A^dagger b = (A^top A)^(-1) A^top b $

=== 矮胖矩阵（行满秩） 

当 $"rank"(A) = m,quad n eq.gt m$，即，各列线性无关，$A A^top$ 可逆：

$ A^dagger = A^top (A A^top)^(-1) $

此时 $A^dagger$ 称为 $A$ 的右逆：

$ A A^dagger = I_m $

#linebreak()

在线性系统中，此时方程少，未知数多，需要从无穷多的解中，找到范数最小的解：

$ x^* = A^dagger = A^top (A A^top)^(-1) b $

=== 秩亏矩阵

不妨设 $m>n$ 且 $"rank"(A) < n$。用 SVD 分解

$ A^dagger = V Sigma^dagger U^top $

== 求最小二乘解

#note[
  设矩阵 $P=A A^dagger$,

  因为 $ P^2 = A A^dagger A A^dagger = A A^dagger = P = P^top $

  ，于是 *$P$ 正交且幂等，是正交投影矩阵[1]*。

  对于 $x^* = A^dagger b$ ，有 $ A x^*= A A^dagger b = P b $

  ，于是，$A x^*$ 是 $b$ 在 $"Col"(A)$ 上的正交投影，
][
  [1]: 这个定理证明比较复杂。

  直观上，对称矩阵 $P$ 存在正交分解 $P=Q Lambda Q^top$ 。 
  由于幂等性，可知特征值 $lambda^2 = lambda$，因此特征值只有 0 或 1。

  在一组正交基下，$P$ 的作用是保留正方向，删掉其余方向。也就是正交投影。
]

== 选取最小范数

$A x = b$ 可能有无穷多解，$A^dagger b$ 可以选取 $norm(x)$ 最小的解。
