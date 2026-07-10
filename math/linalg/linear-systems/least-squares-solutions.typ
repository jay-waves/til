#import "../../../appx/theme.typ" : tufte, note
#show: tufte

#let vec(x) = $upright(bold(#x))$

= 最小二乘解

线性方程组 $A upright(bold(x)) = upright(bold(b))$ 无解时, 最小二乘法用于找到最小化误差平方和的近似解.

#image("../../../attach/least-squares-solution.webp", width: 50%)

将图中平面视为矩阵 $A$ 的列空间, 蓝色向量为 $vec(b)$. 如果 $vec(b)$ 位于 $A$ 列空间之中, 非齐次线性方程组有解; 如果 $vec(b)$ 不位于 $A$ 列空间之中, 其近似解为 $vec(b)$ 在 $A$ 列空间中的投影. 问题等价于求 $A$ 列空间向量 $A vec(x)$ (基底为 $A$, 坐标为 $vec(x)$), 使两向量距离最小: 

$ upright("min") | vec(x) - vec(b) |^2 $

$vec(x)$ 为 $vec(b)$ 投影时, 向量 $vec(x) - vec(b)$ 和 $A ( vec(a)_1 \, . . . vec(a)_n )$ 列空间垂直: 
$
forall vec(a)_i in A \, med ( A vec(x) - vec(b) \, med vec(a)_i ) = 0
$

等价于: $ A^T ( vec(b) - A vec(x) ) = 0 $

解得: $ vec(x) = ( A^T A )^(- 1) A^T vec(b) $

= 另一问题背景

线性回归中, 为了找到数据点 $( x_1 \, y_1 ) \, ( x_2 \, y_2 ) \, dots.h \, ( x_n \, y_n )$ 间的关系, 我们建立线性模型: 

$ y_i = a_1 x_(i 1) + a_2 x_(i 2) + dots.h + a_m x_(i m) + epsilon.alt_i $

其中 $a_1 \, a_2 \, dots.h \, a_m$ 是要求解的参数, $epsilon.alt_i$ 是误差项. 上式用矩阵表示为:


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
