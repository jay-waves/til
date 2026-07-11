#import "../../appx/theme.typ" : tufte, note, theorem, lemma, corollary, definition, proof

#show: tufte

= 对称矩阵

$A^top = A$, 所有特征值都为实数.

$( sigma ( alpha ) \, beta ) = ( alpha \, sigma ( beta ) )$

对于任何方阵 $X$, $X + X^top$ 是一个对称矩阵, $X - X^top$ 是一个斜对称矩阵.

= 正交矩阵

$Q^top Q = Q Q^top = E_n$

#corollary[
  - $| det Q | = 1$
  - $Q^top = Q^(- 1)$
]

描述空间中一组正交的单位基底. 乘向量时 (表线性变换时), 使向量旋转, 长度不变; $det Q = - 1$ 时使向量镜像旋转 (改变手性).

设 $Q = [ q_1 \, q_2 \, dots.h \, q_n ]$, 那么有: 

$
q_i^T q_j = {1 \, quad i = j\
0 \, quad i eq.not j
$


== 酉矩阵

酉矩阵 (幺正矩阵, Unitary Matrix) 是复数域 $bb(C)$ 上的正交矩阵, 记为 $U$.

- $U^top U = U U^top = E_n$
- $U^(- 1) = U^top$
- $| lambda_n | = 1$
- $|U|=1$
- 酉矩阵不改变向量点积: 

$ ( U upright(bold(x)) ) dot ( U upright(bold(y)) ) = upright(bold(x dot y)) $

- 若 U 和 V 都是酉矩阵, 则 $U V$ 也是酉矩阵: 

$ ( U V )^top ( U V ) = ( U V ) ( U V )^top = E_n $


== 正规矩阵

*正规矩阵 (normal matrix)* 是指: $A^H A = A A^H$, 其中 $A^H$ 是 $A$ 的共轭转置.

例子如 #link("向量分析/内积空间.typ")[Hermite 矩阵], 酉矩阵, 实对称矩阵.

#line(length: 100%)

= 实对称矩阵的相似对角化

如果 $A = A^top$, 且属于实数域, 则称 $A$ 为实对称矩阵.

#theorem[如果 $A$ 是实对称矩阵, 那么 $A$ 的特征值全部是实数.]

#proof[...]

#theorem[如果 $A$ 是实对称矩阵, 那么 $A$ 的几何重数一定等于代数重数.]

#proof[该定理是说, $A$ 特征空间维数一定等于代数重数, 即能找到代数重数个数的线性无关向量]

#theorem[如果 $A$ 是实对称矩阵, 那么 $A$ 不同特征的特征向量一定正交.]

#proof[
  设不同特征值 $lambda_1 \, lambda_2$, 有 $A xi_1 = lambda_1 xi_1 \, A xi_2 = lambda_2 xi_2$.  
  有: $lambda_2 xi_1^top xi_2 = xi_1^top A xi_2 = xi_1^top A^top xi_2 = ( A xi_1 )^top xi_2 = lambda_1 xi_1^top xi_2$  
  因为 $lambda_1 eq.not lambda_2$, 所以 $xi_1 perp xi_2$
]

== 谱定理

#theorem[
  实对称矩阵 $A_(n times n)$ 可以被正交矩阵相似对角化:


  $
  Q^top A Q = Q^(- 1) A Q = Lambda = upright("diag") ( lambda_1 \, lambda_2 \, dots.h.c \, lambda_k )
  $
]


其中, 
- $upright("diag")()$ 表示对角矩阵 (Dignomal Matrix), 参数为对角元素.
- $k lt.eq n$, 因为可能存在重根, 重根按重数记入 $Lambda$ 参数.

根据#link("矩阵相似.typ")[矩阵相似对角化原理], $Q = ( xi_1 \, xi_2 \, dots.h \, xi_n )$, $Q^top = Q^(- 1) = ( xi_1^top \, xi_2^top \, dots.h \, xi_n^top )^top$. 于是有: 

$
A = Q Lambda Q^top = [ xi_1 \, xi_2 \, dots.h \, xi_n ] mat(delim: "[", lambda_1; #none, lambda_2; #none, , dots.down; #none, , , lambda_n) mat(delim: "[", xi_1^top; xi_2^top; dots.v; xi_n^top) = lambda_1 xi_1 xi_1^top + lambda_2 xi_2 xi_2^top + dots.h + lambda_n xi_n xi_n^top
$


$
xi_1 xi_1^top + xi_2 xi_2^top + dots.h + xi_n xi_n^top = Q Q^top = E_n
$


#linebreak()

设向量 $upright(bold(x))$, 经过 $A$ 线性变换后, 可以表示为 $A upright(bold(x)) = Q^top Lambda Q upright(bold(x))$, 这说明, 实对称矩阵描述的线性变换是正交的, 通过旋转 ($Q upright(bold(x))$) 将向量从#link("向量分析/空间基底变换.typ")[默认标准正交单位基底]调整为新的正交单位基底, 然后进行正交变换 ($Lambda upright(bold(x))$), 然后再变换回旧基底 ($Q^top upright(bold(x))$). 

#linebreak()

$A = Q Lambda Q^T$ 也被称为*正交分解*, 属于特征值分解的一种特例. 当 $A$ 不满秩时, 也能对角化, 但是存在特征值 $lambda = 0$.

#theorem[在实数域上, A 可正交对角化, 当且仅当, A 是实对称矩阵 ($A^T = A$).]

#theorem[在复数域上, A 可酉对角化, 当且仅当, A 是正规矩阵 ($A^H A = A A^H$).]

== 谱半径

设矩阵 $A_(n times n)$ 的特征值为 $lambda_1 \, lambda_2 \, dots.h \, lambda_n$, 那么 $A$ 的*谱半径*定义为: 
$
rho ( A ) = max | lambda_i |
$



= 实对称矩阵的交换性

#lemma[对角矩阵 $Lambda$ 和 $B$ 可交换, 当且仅当 $B$ 是和 $Lambda$ 结构对应的分块对角矩阵.]

#proof[
  不妨设 $Lambda$ 形式如下, $lambda_i$ 值各不相同: 
$
Lambda = mat(delim: "(", lambda_1 I_(k_1), 0, dots.h.c, 0; 0, lambda_2 I_(k_2), dots.h.c, 0; dots.v, dots.v, dots.down, dots.v; 0, 0, dots.h.c, lambda_m I_(k_m))
$

将 $B$ 按 $Lambda$ 格式分块为: 
$
B = mat(delim: "(", B_11, B_12, dots.h.c, B_(1 m); B_21, B_22, dots.h.c, B_(2 m); dots.v, dots.v, dots.down, dots.v; B_(m 1), B_(m 2), dots.h.c, B_(m m))
$

那么:

$
Lambda B = mat(delim: "(", lambda_1 B_11, lambda_1 B_12, dots.h.c, lambda_1 B_(1 m); lambda_2 B_21, lambda_2 B_22, dots.h.c, lambda_2 B_(2 m); dots.v, dots.v, dots.down, dots.v; lambda_m B_(m 1), lambda_m B_(m 2), dots.h.c, lambda_m B_(m m))
$


$
B Lambda = mat(delim: "(", lambda_1 B_11, lambda_2 B_12, dots.h.c, lambda_m B_(1 m); lambda_1 B_21, lambda_2 B_22, dots.h.c, lambda_m B_(2 m); dots.v, dots.v, dots.down, dots.v; lambda_1 B_(m 1), lambda_2 B_(m 2), dots.h.c, lambda_m B_(m m))
$


要使 $Lambda B = B Lambda$, 需要 $B_(i j) = 0 upright(", if ") i eq.not j$, 所以 $B$ 最终形式为:


$
B = mat(delim: "[", B_1, 0, dots.h.c, 0; 0, B_2, dots.h.c, 0; dots.v, dots.v, dots.down, dots.v; 0, 0, dots.h.c, B_m)
$
]

#theorem[如果 $A, B$ 皆为实对称矩阵, $A B = B A$, 当且仅当存在正交矩阵 $Q$, 可将 $A, B$ 同时相似对角化: $Q^top A Q$, $Q^top B Q$]

> 这个定理很重要, 可以将条件 $A,B$ 为实对称矩阵泛化为 $A,B$ 是可对角化的.

#proof[
  *必要性*: 如果 $A, B$ 可被矩阵 $Q$ 同时相似对角化, 即 $A, B$ 有相同特征空间, 那么 ${} A B = B A$.  
  不妨令 $Q^(- 1) A Q = Lambda_1 \, Q^(- 1) B Q = Lambda_2$, 则有: 
$
A B = Q Lambda_1 Q^(- 1) Q Lambda_2 Q^(- 1) = Q Lambda_1 Lambda_2 Q^(- 1)\
B A = Q Lambda_2 Q^(- 1) Q Lambda_1 Q^(- 1) = Q Lambda_2 Lambda_1 Q^(- 1)
$
   
  由于对角矩阵可交换: $Lambda_1 Lambda_2 = Lambda_2 Lambda_1$, 所以 $A B = B A$. 必要性得证.
]

#note[[^1]: 该证明来自: R. A. Horn and C. R. Johnson, *Matrix Analysis*, 2nd ed. Cambridge, UK: Cambridge University Press, 2013, p. 62, Theorem 1.3.12.]

#proof[
  *充分性*[^1]: $A,B$ 为实对称矩阵, 如果 $A B = B A$, 那么 $A, B$ 可被同一矩阵相似对角化.

矩阵 $A \, B in M_n$ 是实对称矩阵, 所以它们是可相似对角化的.   
对 $A$ 相似对角化, 不妨设 $Q^top A Q = Lambda_A$, $Q^top B Q = C$, 注意 $C$ 不一定是对角矩阵.  

$
Q^(- 1) A Q = Lambda_A = mat(delim: "[", mu_1 E_(n_1), 0, dots.h.c, 0; 0, mu_2 E_(n_2), dots.h.c, 0; dots.v, dots.v, dots.down, dots.v; 0, 0, dots.h.c, mu_d E_(n_d))
$
, 其中 $mu_1 \, mu_2 \, dots.h \, mu_d$ 是不同的特征向量, 令 $n_i$ 是 $mu_i$ 的重数.  
因为 $A B = B A$, 所以 $C$ 应该是和 $Lambda_A$ 相对应的块对角矩阵 
$
Q^(- 1) B Q = C = mat(delim: "[", B_1, 0, dots.h.c, 0; 0, B_2, dots.h.c, 0; dots.v, dots.v, dots.down, dots.v; 0, 0, dots.h.c, B_d)
$
, 其中 $B_i in M_(n_i)$.  
因为 $B$ 可相似对角化, 所以 $Q^(- 1) B Q$ 仍可以相似对角化, $B_i$ 也可以相似对角化[^2]. 令 $T_i in M_(n_i)$ 满足 $T_i^(- 1) B_i T_i = Lambda_i$, 且是非奇异的.  

#note[[^2]: 分块对角矩阵 $B = B_1 xor B_2 xor dots.h xor B_d$ 可相似对角化, 当且仅当每一个分块 $B_i$ 都是可相似对角化的. 该结论可能不那么显然, 可参考书中证明, 证明使用归纳假设法."]

定义矩阵 
$
T = mat(delim: "[", T_1, 0, dots.h.c, 0; 0, T_2, dots.h.c, 0; dots.v, dots.v, dots.down, dots.v; 0, 0, dots.h.c, T_d)
$
 
由于 $T_i^(- 1) mu_i E_(n_i) T_i = mu_i E_(n_1)$, 所以 $T^(- 1) Q^(- 1) A Q T = Lambda_A$  
并且 
$
T^(- 1) C T = T^(- 1) Q^(- 1) B Q T = mat(delim: "[", T_1^(- 1) B_1 T_1, 0, dots.h.c, 0; 0, T_2^(- 1) B_2 T_2, dots.h.c, 0; dots.v, dots.v, dots.down, dots.v; 0, 0, dots.h.c, T_d^(- 1) B_d T_d) = Lambda_B
$
 

所以存在可逆矩阵 $T Q$, 使得矩阵 $A, B$ 可同时相似对角化.
]

#lemma[$A B = B A$, 当且仅当 $A, B$ 有公共的特征向量.]

#proof[

任取 $A$ 的一个特征值 $lambda$, 设其对应的特征子空间 $bb(V)_lambda = {p | A p = lambda p}$, ${ alpha_1 \, alpha_2 \, dot.op dot.op dot.op \, alpha_m }$ 是 $bb(V)_lambda$ 的一个基.  
由 $A B = B A$ 知, $A ( B alpha_i ) = B ( A alpha_i ) = lambda B alpha_i in bb(V)_lambda \, ( i = 1 \, 2 \, dots.h \, m )$.  
于是存在数 $k_(i j) \, ( i \, j = 1 \, 2 \, dot.op dot.op dot.op \, m )$, 使得

$
B alpha_i = k_(1 i) alpha_1 + k_(2 i) alpha_2 + dot.op dot.op dot.op + k_(m i) alpha_m \, med i = 1 \, 2 \, dot.op dot.op dot.op \, m .
$
  
记 $C = [alpha_1 \, alpha_2 \, dot.op dot.op dot.op \, alpha_m]$, $K = [k_(i j)]_(m times m)$, 则 $B C = C K$．  
设 $mu$ 为 $K$ 的一个特征值, $beta$ 是 $K$ 的对应于 $mu$ 的特征向量, 即 $beta eq.not 0$, $K beta = mu beta$.  
又取 $p = C beta$, 则 $p in bb(V)_lambda$ , 所以 $p$ 是 $A$ 的对应于 $lambda$ 的特征向量,   
于是 $B p = B ( C beta ) = C K beta = C ( mu beta ) = mu p$.   
于是 $p$ 也是 $B$ 的对应于 $mu$ 的特征向量, 故 $A$ 与 $B$ 有公共的特征向量. 
(不过难以确认 $K$ 是否有足够数量的线性无关特征向量)
]


#theorem[如果 $A, B$ 皆为正定矩阵, $A B = B A$, 当且仅当 $A B$ 也是正定的]

#proof[

因为 $A, B$ 正定, 所以存在可逆矩阵 $P_1 \, P_2$, 使得 $A = P_1^top P_1 \, B = P_2^top P_2$  
于是 $A B = ( P_1^top P_1 ) ( P_2^top P_2 ) = P_2^(- 1) med ( P_2 P_1^top ) ( P_1 P_2^top ) med P_2 = P_2^(- 1) med ( P_1 P_2^top )^top ( P_1 P_2^top ) med P_2 = P_2^(- 1) C^top C P_2$  
矩阵 $C^top C$ 正定, 而 $A B tilde.op C^T C$, 所以 $A B$ 正定.
]
