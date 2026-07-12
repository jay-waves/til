#import "../../appx/theme.typ" : tufte, theorem, lemma, corollary, proof

#show: tufte
#let bmat(..args) = math.mat(delim: "[", ..args)


= 矩阵的逆

对于矩阵 $B$, 有 $B dot.op B^(- 1) = I$. $B^(- 1)$ 表示和 $B$ 相反的线性变换.

= 初等变换矩阵 

用矩阵形式表示对一个矩阵的初等变换, 记为 $E$, 左乘目标矩阵是对其行做同等变换, 右乘目标矩阵是对其列做同等变换.

*行交换:* 交换第1行和第2行

$
bmat(0, 1, 0; 1, 0, 0; 0, 0, 1)
$

*行倍乘:* 第2行乘2

$
bmat(1, 0, 0; 0, 2, 0; 0, 0, 1)
$

*行倍加:* 第一行的2倍加到第3行

$
bmat(1, 0, 0; 0, 1, 0; 2, 0, 1)
$

= 求解逆矩阵

== 初等变换方法

行初等变换 $bmat(B, I)$ 为 $bmat(I, B^(- 1))$, 目的是将左侧 $B$ 变为 $I$, 与此同时右侧 $I$ 会变成 $B^(- 1)$.

原理是: 可逆矩阵 $B$ 行空间能张成整个 $n$ 维空间, 因而从存在一系列初等变换矩阵, 能够将可逆矩阵 $B$ 行变换为单位矩阵 $I$ ($n$ 维空间标准正交基底).


$
P_1 dot.op P_2 dot.op dots.h.c P_k dot.op bmat(B, I) = bmat(( product_(i = 1)^k P_i ) dot.op B, ( product_(i = 1)^k P_i ) dot.op I) = bmat(I, B^(- 1))
$

== 伴随矩阵法

适用于简单二阶矩阵, 否则伴随矩阵会变得非常复杂. 原理如下:

1. $A dot.op A^(*) = | A | dot.op I$
2. 同乘 $A^(- 1)$, 有 $A^(- 1) = frac(A^(*), | A |)$
3. 对于二阶矩阵 $A = bmat(a, b; c, d)$, 有 $A^(*) = bmat(d, - b; - c, a)$

