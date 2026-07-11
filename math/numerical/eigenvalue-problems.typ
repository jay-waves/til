#import "../../appx/theme.typ": tufte, note

#show: tufte

#let bmat(..args) = $mat(delim: "[", ..args)$
#let vmat(..args) = $mat(delim: "|", ..args)$

= 特征值求解问题

给定 $A$, 求 $A x = lambda x$.

== 幂法
幂法用于计算矩阵的模最大的特征值和其特征向量.

这里假设 $A_(n times n)$ 有 $n$ 个线性无关特征向量 $x_i$, 其对应特征值满足: $ | lambda_1 | > | lambda_2 | gt.eq | lambda_3 | gt.eq dots.h gt.eq | lambda_n | $

对于非零向量 $u_0$, 存在非零数组 $a_1 , a_2 , dots.h , a_n$ 满足:

$ u_k = A thin u_(k - 1) = A^2 u_(k - 2) = dots.c = A^k u_0 =\
alpha_1 A^k x_1 + alpha_2 A^k x_2 + dots.c + alpha_n A^k x_n =\
alpha_1 lambda_1^k x_1 + alpha_2 lambda_2^k x_2 + dots.c + alpha_n lambda_n^k x_n =\
lambda_1^k [ med alpha_1 x_1 + alpha_2(lambda_2 / lambda_1)^k x_2 + dots.c + alpha_n(lambda_n / lambda_1)^k x_n med ] $

#emph[如果 $a_1 eq.not 0$, 当 $k$ 充分大时, 有:] $ u_k approx lambda_1^k alpha_1 x_1 $

因为 $x_1$ 是矩阵 $A$ 的特征向量, 那么 $u_k$ 也是 $A$ 的特征向量.

=== 归一化
为了避免 $u_k$ 的模过大 (当 $| lambda_1 | > 1$) 或过小 (当 $| lambda_1 | < 1$), 实际计算时, 在每一步对 $u_k$ 进行归一化:

$ u_k = frac(A u_(k - 1), norm(u_(k - 1))) = frac(A^2 u_(k - 2), norm(A u_(k - 2))) = dots.h = frac(A^k u_0, norm(A^(k - 1) u_0)) $

记: $ u_k = frac(A u_(k - 1), norm(u_(k - 1))) = A y_(k - 1) $

那么 $ y_k = frac(u_k, norm(u_k)) = frac(A^k u_0, norm(A^k u_0)) $

当 $k arrow.r oo$ 时, 若 $lambda_1 > 0$, 有 $y_k arrow.r frac(alpha_1 x_1, norm(alpha_1 x_1))$. 那么 $y_k$ 可近似地作为 $A$ 的特征向量.

如果范数取 #emph[二范数]:

取 $ beta_k = y_(k - 1)^top u_k = y_(k - 1)^top A y_(k - 1) $

那么 $ lim_(k arrow.r oo) beta_k = frac(alpha_1 x_1^top, norm(alpha_1 x_1)_2) A frac(alpha_1 x_1, norm(alpha_1 x_1)_2) = frac(a_1^2 x_1^top x_1, norm(a_1 x_1)_2^2) lambda_1 = lambda_1 $

当 $| beta_k - beta_(k - 1) | / | beta_k | lt.eq epsilon$ (允许误差) 时, 迭代终止. 此时 $beta_k$ 作为 $lambda_1$ 近似值, $y_(k - 1)$ 作为 $A$ 的近似特征向量.

如果范数取 #emph[无限范数]:

取 $ beta_k = frac(e_r^top u_k, e_r^top y_(k - 1)) = frac(e_r^top A y_(k - 1), e_r^top y_(k - 1)) $

其中 $u_(k - 1)$ 的第 $r$ 个分量为模最大的分量. $e_r$ 是 $n$ 维基本单位向量, 它的第 $r$ 个分量为 $1$, 其余分量为零.

当 $y_k arrow.r frac(alpha_1 x_1, norm(alpha_1 x_1))$, 也有 $ lim_(k arrow.r oo) beta_k = frac(e_r^top A x_1, e_r^top x_1) $

=== 反幂法
假设非奇异矩阵 $A$ 的特征值满足: $ | lambda_1 | gt.eq | lambda_2 | gt.eq dots.h gt.eq | lambda_(n - 1) | > | lambda_n | $

要求 $A$ 的模最小的特征值 $lambda_n$ 及特征向量.

因为 A 非奇异, 因此 $lambda_i eq.not 0$. 由 $ A x_i = lambda_i x_i $, 得到 $ A^(- 1) x_i = 1 / lambda_i x_i $

此时 $1 / lambda_n$ 是 $A^(- 1)$ 的按模最大的特征值. 复用上述幂法即可. 反幂法多出了求逆的步骤.

=== 移位反幂法
已知 $lambda_n$ 后, 取 $mu = lambda_n + epsilon$. 此时反幂法 $( A - mu I )^(- 1)$ 的特征值为 $lambda'_i = frac(1, lambda_i - mu)$.

此时, $lambda_(n - 1)$ 对应的 $lambda'$ 最大, 带入幂法中即可求出 $lambda_(n - 1)$.

如果不用移位法, 也可以通过正交投影消除 $lambda_n$ 的影响: $ B = A - lambda_n v_n v_n^top $, 其中 $v_n$ 是 $lambda_n$ 对应特征向量, 此时 $B$ 的最小特征值是 $A$ 的次小特征值 $lambda_(n - 1)$

== 盖尔圆盘定理 (Gershgorin Circle)
对于矩阵 $A = ( a_(i j) ) in bb(C)^(n times n)$, 定义每一行的#emph[盖尔圆盘]:

$ D_i = lr({ z in bb(C): |z - a_(i i)| lt.eq sum_(j eq.not i) |a_(i j)| }) $

定理: #strong[矩阵 $A$ 的所有特征值都包含在所有盖尔圆盘的并集中] $sigma(A) subset.eq union.big_(i = 1)^n D_i$

#strong[证明]:

取特征方程 $A x = lambda x$ 的最大分量 $x_k$, 比较:

$ | lambda - a_(k k) | = lr(|sum_(j eq.not k) a_(k j) x_j|) lt.eq sum_(j eq.not k) | a_(k j) norm(x_j | lt.eq sum_(j eq.not k) | a_(k j)) x_k | $

消掉 $| x_k |$, 得到:

$ | lambda - a_(k k) | lt.eq sum_(j eq.not k) | a_(k j) | $

$square.filled.medium$

=== 推论1
#strong[若有一组 $k$ 个盖尔圆盘的并集, 与其余圆盘完全不相交, 则该并集中恰好包含 $k$ 个特征值. (按代数重数计)].

=== 推论2
#strong[若对所有 $i$, 均满足严格对角占优: $ | a_(i i) | > sum_(j eq.not i) | a_(i j) | $, 则所有圆盘不包含零点, 于是 $0 in.not sigma(A)$, 矩阵必然可逆.]

== Jacobi
…

== QR 分解法
=== Householder 矩阵
#strong[定义: $H$ 是对称正交矩阵, 满足 $H = I - 2 v v^top$, 其中 $v$ 是单位向量.]






