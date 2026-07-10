#import "../../appx/theme.typ" : tufte

#show: tufte

= 矩阵的秩

矩阵的秩指矩阵列 (行) 空间线性无关向量的个数.

== 定理1

*矩阵行秩等于列秩*

*证明:[^1]*

[^1]: 参考#link("https://zh.wikipedia.org/wiki/线性代数基本定理")[维基百科]的三种证明

#linebreak()

*证明2:*

对矩阵 $M_(m times n) ( R )$, 不妨假设 $m > n$.   
对于行空间, $M$ *最多*能提供 $n$ 个线性无关的向量, 因为向量空间维数为 $n$.  
对于列空间, $M$ 也*最多*提供 $n$ 个线性无关的向量, 因为 $M$ 最多有 $n$ 个列向量. 相当于 $m$ 维空间中的 $n$ 维子空间.  
以上证明了*引理: $upright("rank") ( A ) lt.eq n$, 即 $upright("rank") ( A ) lt.eq upright("min") { m \, med n }$. 其中 $upright("rank")(A)$ 是行秩或列秩.*

对于矩阵 $M$, 行变换不会改变其行空间, 列变换也不会改变其列空间, 即变换不影响矩阵的秩.  
将 $M$ 不断行变换化简, 并消除零行, 得到最简 $r$ 行矩阵, 是 $M$ 行空间的最大线性无关组, 此时 $upright("rank")(A)_("rows") = r$.  必然有, $r lt.eq n$, 因为行向量维数为 $n$.  
行变换化简后, 行空间最多只有 $r$ 个线性无关的向量, 由上述引理知, $upright("rank")(A)_("columns") lt.eq r$.   
由假设反证法可知, 此时列空间不可能再通过列变换压缩为比 $r$ 更低维度的空间, 否则行空间秩会比 $r$ 更小. 所以 ==$upright("rank") ( A ) equiv r$==.

#linebreak()

== 定理2

对于齐次线性方程组 $A_(m times n) upright(bold(x)) = upright(bold(0))$, 其解空间 $nothing_A$ 和 $A$ 列空间的秩的关系为:

$
upright("rank")(A) + upright("rank")(upright("null")_A) = n
$

*推论:* 若 $A B = 0$, 则 $r ( A ) + r ( B ) lt.eq n$

== 定理3

$
upright("rank")(A B) <= upright("min")(upright("rank")(A), upright("rank")(B))
$

这反映了矩阵变换不会使空间增广, 只会使秩减小或不变.

== 定理4

$
upright("rank") ( A plus.minus B ) lt.eq upright("rank") ( A ) + upright("rank") ( B )
$

== 定理5

$
r ( nothing_(A B) ) lt.eq r ( nothing_A ) + r ( nothing_B )
$

*证明:*

#link("线性方程组/线性方程组的解.typ")[线性方程组] $B upright(bold(x)) = 0$ 的解空间, 属于 $A B upright(bold(x)) = 0$ 的解空间, 即 $nothing_B subset.eq nothing_(A B)$.  
当 $A B upright(bold(x)) = 0$ 的解不是 $B upright(bold(x)) = 0$ 的解, 即 $B upright(bold(x)) eq.not 0$, 则 $B upright(bold(x))$ 属于 $A upright(bold(x)) = 0$ 的解空间.  
因此 $r ( nothing_(A B) ) - r ( nothing_B ) lt.eq r ( nothing_A )$.

$square.filled.medium$

== 定理6 

$
upright("rank") ( A B ) gt.eq upright("rank") ( A ) + upright("rank") ( B ) - n
$

*证明:*

因为 $r ( nothing_(A B) ) lt.eq r ( nothing_A ) + r ( nothing_B )$  
所以由定理一: $n - r ( A B ) lt.eq 2 n - r ( A ) - r ( B )$  
即: $r ( A B ) gt.eq r ( A ) + r ( B ) - n$

$square.filled.medium$

当且仅当 $nothing_A perp nothing_B$ 时, 有 $r ( nothing_(A B) ) = r ( nothing_A ) + r ( nothing_B )$

== 定理7

$r ( A A^T ) = r ( A^T A ) = r ( A ) = r ( A^T )$

*证明*: 

即证 $A upright(bold(x)) = 0$ 和 $A^T A upright(bold(x)) = 0$ 是同解方程组  
充分性: 由 $A upright(bold(X)) = 0$, 得 $A^T A upright(bold(x)) = A^T upright(bold(0)) = 0$  
必要性: 由 $A^T A upright(bold(x)) = 0$, 得 $upright(bold(x^T)) A^T A upright(bold(x)) = ( A upright(bold(x)) )^T A upright(bold(x)) = bar.v.double A upright(bold(x)) bar.v.double^2 = 0$, 所以 $A upright(bold(x)) = 0$

$square.filled.medium$

实质上 $C ( A^T A ) = R ( A^T A ) = C ( A^T ) = R ( A )$


= 矩阵的逆

对于矩阵 $B$, 有 $B dot.op B^(- 1) = I$. $B^(- 1)$ 表示和 $B$ 相反的线性变换.

== 初等变换矩阵 

用矩阵形式表示对一个矩阵的初等变换, 记为 $E$, 左乘目标矩阵是对其行做同等变换, 右乘目标矩阵是对其列做同等变换.

*行交换:* 交换第1行和第2行


$
mat(delim: "[", 0, 1, 0; 1, 0, 0; 0, 0, 1; #none)
$


*行倍乘:* 第2行乘2


$
mat(delim: "[", 1, 0, 0; 0, 2, 0; 0, 0, 1; #none)
$


*行倍加:* 第一行的2倍加到第3行


$
mat(delim: "[", 1, 0, 0; 0, 1, 0; 2, 0, 1; #none)
$

== 求解逆矩阵

=== 1 初等变换方法

行初等变换 $mat(delim: "[", B, I)$ 为 $mat(delim: "[", I, B^(- 1))$, 目的是将左侧 $B$ 变为 $I$, 与此同时右侧 $I$ 会变成 $B^(- 1)$.

原理是: 可逆矩阵 $B$ 行空间能张成整个 $n$ 维空间, 因而从存在一系列初等变换矩阵, 能够将可逆矩阵 $B$ 行变换为单位矩阵 $I$ ($n$ 维空间标准正交基底).


$
P_1 dot.op P_2 dot.op dots.h.c P_k dot.op mat(delim: "[", B, I) = mat(delim: "[", ( product_(i = 1)^k P_i ) dot.op B, ( product_(i = 1)^k P_i ) dot.op I) = mat(delim: "[", I, B^(- 1))
$

=== 2 伴随矩阵法

适用于简单二阶矩阵, 否则伴随矩阵会变得非常复杂. 原理如下:

1. $A dot.op A^(*) = | A | dot.op I$
2. 同乘 $A^(- 1)$, 有 $A^(- 1) = frac(A^(*), | A |)$
3. 对于二阶矩阵 $A = mat(delim: "[", a, b; c, d)$, 有 $A^(*) = mat(delim: "[", d, - b; - c, a)$

= 矩阵等价

如果存在可逆矩阵 $P$ 和 $Q$, 使得 $P A Q = B$, 那么称矩阵 $A$ 和 $B$ 等价, 记 $A tilde.equiv B$.

$P A = B$, 说明 $A$ 和 $B$ 的行向量组等价, 左乘可逆矩阵是对行向量组的线性组合; $A P = B$, 说明 $A$ 和 $B$ 的列向量组等价, 右乘可逆矩阵是对列向量组的线性组合. $P A Q = B$ 没有上述结论, 这说明矩阵等价不意味着两矩阵的向量组等价, 只能说明两者秩相同.
