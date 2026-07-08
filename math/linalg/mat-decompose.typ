
#import "../../template.typ": tufte, meta, note

#show: tufte

#let bmat(..args) = math.mat(delim: "[", ..args)

#meta[
- subtitle: 矩阵分解
]

#image("../../attach/matrix-world.webp")

#note(
  "图片来自 *the Art of Linear Algebra*
  翻译版本请见 ../../attach/matrix-world-zh.webp"
)

= 矩阵分解的类型

矩阵有五种分解: 
1. 行列空间分解 (满秩分解)
2. #link("../numerical/线性方程组的解法.typ")[LU 分解]
3. #link("向量分析/直和与投影.typ")[QR 分解]
4. #link("矩阵相似.typ")[特征值分解]
5. 奇异值分解

特征值分解中, 包含:
- #link("若尔当分解.typ")[Jordan 分解]: 相似变换, $A = S J S^(- 1)$, 其中 $S$ 是可逆矩阵.
- Shcur 分解: 酉/正交相似, $A = U T U^H$, 其中 $U$ 是酉矩阵.
- #link("对称矩阵.typ")[谱定理 (正交分解)]: 酉/正交对角化, $A = U Lambda U^H$, 其中 $U$ 是酉矩阵, 并且要求 $A$ 是正规矩阵.

== 满秩分解

$A in bb(C)_r^(m times n) ( r > 0 )$, 存在列满秩矩阵 $B in bb(C)_r^(m times r)$ 和行满秩矩阵 $C in bb(C)_r^(r times n)$ 使得: $A = B C$

#note[等价，也成为矩阵相抵，详见 math/linalg/readme.typ ] #linebreak()

*证明*: $A$ 和 $bmat(I_r, 0; 0, 0)$ *等价*, 因此存在可逆矩阵 $P,Q$, 满足: 

$ A = P bmat(I_r, 0; 0, 0) Q = P bmat(I_r; 0) bmat(I_r, 0) Q $

那么有 $B = P bmat(I_r; 0)$ 以及 $C = bmat(I_r, 0) Q$, 即 $A = B C$. 

#align(right)[$square.filled$]

这里 $B$ 是 $A$ 的列空间基向量. 显然, 满秩分解并不唯一.

=== 满秩分解的求法

不妨设 $A$ 的秩为 $r$.

行变换, 将 $A$ 化简为行最简矩阵: $ P A = R $


取出 $R$ 的非零行作为 $C$: 

$ P A = mat(delim: "[", I_r; 0) C $

因此有: 

$ A = P^(- 1) mat(delim: "[", I_r; 0) C = B C $

== Schur 分解

#note("酉矩阵详见 ./symmetric.typ")

*任意*矩阵 $A in bb(C)^(n times n)$ 皆存在*酉矩阵* $U$, 使得 

$ U^H A U = T $

, 其中 $T$ 是上三角矩阵, 并且 $T$ 的对角线元素正好是 $A$ 的全部特征值.

任意复矩阵都能通过单位正交变化, 变为上三角矩阵. 而上三角矩阵的对角元一定是其特征值.

=== Schur 分解的数值求法

*算法*:

对矩阵 $A$ 执行一次 #link("向量分析/直和与投影.typ")[QR] 分解: $A = Q R$, 然后构造 $A_1 = R Q$.

接着不断 QR 迭代: 

$ 
  A_1 = Q_1, quad A_2 = R_1 Q_1 \
  A_2 = Q_2, quad A_3 = R_2 Q_2 \
  \ dots.h \
  A_k = Q_k , quad A_(k+1) = R_k Q_k
$


最终 $A_n arrow.r T$, 算法逼近一个上三角矩阵 $T$, 而 $U = Q_0 Q_1 Q_2 dots.h Q_n$, 最终得到 Schur 分解: 
$ A = U T U^H $

*证明*: 

$A_k = R_(k - 1) Q_(k - 1) = Q_(k - 1)^H A_(k - 1) Q_(k - 1)$ 这是一次酉相似变换, 不改变特征值. 

其他证明看不懂, 先略.

== 奇异值分解

奇异值分解 (Singular Value Decomposition, SVD). 对域 $K$ 上的任意矩阵 $A_(m times n)$ 可分解为: 

$ A = U med Sigma med V^top $

- $U$ 是 $m times m$ 的酉矩阵, 是 $A A^top$ 的特征向量.
- $V$ 是 $n times n$ 的酉矩阵, 是 $A^top A$ 的特征向量.
- $Sigma$ 是 $m times n$ 的非负实数对角矩阵, 对角元素称为奇异值, 是 $A A^top$ 和 $A^top A$ 的特征值的非负平方根, 并和 $U$ 和 $V$ 的行向量相对应.

=== 奇异值和奇异向量

非负实数 $sigma$ 是 $A$ 的奇异值, 仅当, 存在单位向量 $u in bb(K)^m \, v in bb(K)^n$ 满足: 
$
A v = sigma u \, A^top u = sigma v
$

称 $u,v$ 为 $sigma$ 的左右奇异向量.

=== 定理一

$A^H A$ 和 $A A^H$ 有完全相同的非零特征值 (含重数).

*证明*: 

假设 $lambda eq.not 0$, $A^H A x = lambda x$

两边左乘 $A$, 得到: $A A^H ( A x ) = lambda ( A x )$

由于 $A x$ 是非零向量, 因此 $lambda$ 也是 $A A^H$ 的特征值. 

#align(right)[$square.filled$]

对于 $A^T A$ 和 $A A^T$ 的零特征值, 其个数由其矩阵维数决定, 因为两个矩阵可能不同型. 

=== 奇异值分解的数值求法

- Householder 变换为双对角矩阵
- 对双对焦矩阵继续 QR 迭代
