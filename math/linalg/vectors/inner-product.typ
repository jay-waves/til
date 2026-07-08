#import "../../../template.typ" : tufte

#show: tufte

= 内积空间

设 $F = bb(R)$, $V$ 是 $F$ 上的线性空间. 若对 $V$ 中任意两向量 $alpha \, beta$, 定义向量内积 $( alpha \, beta ) in F$, 使得 $forall x \, y \, z in V$ 和 $k in F$ 满足:
1. 共轭对称性 $( x \, y ) = overline(( y \, x ))$
2. 可加性: $(x+y,z)=(x,z)+(y,z)$
3. 齐次性: $(k x,y)=k(x,y)$, $( x \, k y ) = overline(k) overline(( y \, x )) = overline(k) ( x \, y )$
4. 正定性: $( x \, x ) gt.eq 0$, 当且仅当 $x=0$ 时等号成立.

此时, $V$ 是一个*内积空间*, $(x,y)$ 称为*内积*. *有线维的实内积空间称为欧几里得空间 (Euclidean Space), 有限维的复内积空间 ($F = bb(C)$) 称为酉空间*. 

> 注意, 向量内积不满足结合律: $( upright(bold(a)) dot.op upright(bold(b)) ) upright(bold(c)) eq.not bold(a) ( upright(bold(b)) dot.op upright(bold(c)) )$, 内积后映射到 $bb(F)$ 而不是向量空间 $bb(V)$

== 例子

$bb(R)^n$ 中, $( x \, y ) = y^T = x_1 y_1 + dots.h + x_n y_n$

$bb(C)$ 中, $( x \, y ) = y^H x = x_1 overline(y)_1 + dots.h + x_n overline(y)_n$. 其中 $y^H = [ overline(y)_1 \, dots.h \, overline(y)_n ]$ 为向量 $y$ 的共轭转置.

注意到, $overline(x^H) = x^T$

== Hermite 矩阵

设 $A in bb(C)^(n times n)$, Hermite 矩阵满足: $A^H = A$; 反 Hermite 矩阵满足: $A^H = - A$. 

对于 Hermite 矩阵 $A$, 定义 $bb(C)$ 上的复二次型: $f ( x ) = x^H A x$.

== 度量矩阵

取 $n$ 维欧氏空间 $V$, 在其中取基底 $epsilon.alt_1 \, epsilon.alt_2 \, dots.h.c \, epsilon.alt_n$, 对于其中任意两个向量: 


$
upright(bold(x)) &= x_1 epsilon_1 + x_2 epsilon_2 + ... + x_n epsilon_n \
upright(bold(y)) &= y_1 epsilon_1 + y_2 epsilon_2 + ... + y_n epsilon_n
$


其中 $x_i \, y_i in F$. 因此由内积拆分性质可知: 


$
( upright(bold(x)) \, upright(bold(y)) ) = sum_(i = 1)^n sum_(j = 1)^n ( epsilon.alt_i \, epsilon.alt_j ) x_i overline(y)_j = upright(bold(y))^H G upright(bold(x))
$


, 其中 $G$ 称为基底 $epsilon.alt_1 \, epsilon.alt_2 \, dots.h.c \, epsilon.alt_n$ 的*度量矩阵 (Gram)*. *内积和度量矩阵是一一对应的*. 

=== 定理1

*度量矩阵是 Hermite 矩阵.*

证明: $( upright(bold(x)) \, upright(bold(y)) ) = y^H G x = overline(( upright(bold(y)) \, upright(bold(x)) )) = overline(x^H G y) = y^H G^H x$

=== 定理2

*度量矩阵是正定的.*

证明: 取 $x=y$, 那么 $( x \, x ) = upright(bold(x))^T G upright(bold(x)) > 0$. 

=== 定理3

*不同基底下的度量矩阵是合同的.*

证明: 对于另一组基底 $eta_1 \, eta_2 \, dots.h.c \, eta_n$, 有过度矩阵满足:


$
( eta_1 \, eta_2 \, dots.h.c \, eta_n ) = ( epsilon.alt_1 \, epsilon.alt_2 \, dots.h.c \, epsilon.alt_n ) C
$


设 $G_2$ 和 $G_1$ 分别是 $eta$ 和 $epsilon.alt$ 基底的度量矩阵. 可得:


$
G_2 = ( eta_1 \, dots.h \, eta_n )^T ( eta_1 \, dots.h \, eta_n ) = [ ( epsilon.alt_1 \, epsilon.alt_2 \, dots.h.c \, epsilon.alt_n ) C ]^T [ ( epsilon.alt_1 \, epsilon.alt_2 \, dots.h.c \, epsilon.alt_n ) C ] = C^T G_1 C
$


所以两个矩阵合同. $square.filled.medium$

== 向量长度

设内积空间 $V$, $forall x in V$, $x$ 的长度为: 
$
bar.v.double x bar.v.double = sqrt(( x \, x ))
$


长度的性质:
1. 正定性: $bar.v.double x bar.v.double lt.eq 0$ 当且仅当 $x=0$ 时有 $bar.v.double x bar.v.double = 0$
2. 齐次性: $bar.v.double k x bar.v.double = | k | bar.v.double x bar.v.double$
3. 平行四边形法则: 
$
bar.v.double x + y bar.v.double^2 + bar.v.double x - y bar.v.double^2 = 2 ( bar.v.double x bar.v.double^2 + bar.v.double y bar.v.double^2 )
$


=== 定理

*三角不等式:* 
$
bar.v.double x + y bar.v.double lt.eq bar.v.double x bar.v.double + bar.v.double y bar.v.double
$


=== 定理

*Cauchy-Schwarz 不等式:* 
$
| ( x \, y ) | lt.eq bar.v.double x bar.v.double bar.v.double y bar.v.double
$
 *当且仅当 $x,y$ 线性相关时等号成立*. 

== 向量夹角

设欧式空间 $V$, 定义 $x,y$ 的夹角为: 
$
a = chevron.l x \, y chevron.r = arccos frac(( x \, y ), bar.v.double x bar.v.double bar.v.double y bar.v.double) in [ 0 \, pi ]
$
