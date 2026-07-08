#import "../../../template.typ" : tufte

#show: tufte

*描述坐标, 先要明确基底, 默认为标准正交单位基底. 坐标不代表向量, 规定基底后, 才代表向量.*

假设:

新基底为列向量集合 ${epsilon_(1), epsilon_(2), epsilon_(3), ..., epsilon_(n)}$, 

旧基底为 ${eta_(1),eta_(2),eta_(3),...,eta_(n)}$ (不妨假设两基底都可以张成列向量空间)

设新基底下某向量为: 
$
y = k_1 epsilon.alt_1 + k_2 epsilon.alt_2 + dots.h + k_n epsilon.alt_n = ( epsilon.alt_1 \, epsilon.alt_2 \, dots.h \, epsilon.alt_n ) ( k_1 \, k_2 \, dots.h \, k_n )^top
$
, 称此时 $y$ 在新基底下的坐标为 $( k_1 \, k_2 \, dots.h \, k_n )^top$.

*尝试将向量 $y$ 转换到旧基底 $eta_1 \, eta_2 \, eta_3 \, dots.h \, eta_n$ 下表示:*

== 将旧基底转化为新基底

新基底本质上仍是空间中的一组向量, 那么用旧基底 $eta_1 \, eta_2 \, eta_3 \, dots.h \, eta_n$ 的坐标语言一定也能描述新基底的位置. 那么就有:

$
epsilon.alt_i = x_(1 i) eta_1 + x_(2 i) eta_2 + . . . + x_(n i) eta_n = ( eta_1 \, eta_2 \, eta_3 \, dots.h \, eta_n ) ( x_(1 i) \, x_(2 i) \, dots.h \, x_(n i) )^top
$


即: $( epsilon.alt_1 \, epsilon.alt_2 \, epsilon.alt_3 \, dots.h \, epsilon.alt_n ) = ( eta_1 \, eta_2 \, eta_3 \, dots.h \, eta_n ) P$, 其中 $P = ( x_(i j) )_(i j)$ 称为过渡矩阵, 将旧基底转化为新基底. 

== 坐标变换

对于新基底 ${epsilon_(1), epsilon_(2), epsilon_(3), ..., epsilon_(n)}$ 下的向量坐标 $upright(bold(x)) = ( k_1 \, k_2 \, dots.h \, k_n )^top$, 其表示的向量为: 


$
( epsilon.alt_1 \, epsilon.alt_2 \, epsilon.alt_3 \, dots.h \, epsilon.alt_n ) ( k_1 \, k_2 \, dots.h.c \, k_n )^top = ( eta_1 \, eta_2 \, dots.h.c \, eta_n ) P ( k_1 \, k_2 \, dots.h.c \, k_n )^top
$


所以该向量在旧基底 ${eta_(1),eta_(2),eta_(3),...,eta_(n)}$ 下的坐标 $upright(bold(y)) = P upright(bold(x))$, 其中 $x$ 为新基底下的坐标, 称为坐标-基底变换公式.

== 将线性变换表示为矩阵

线性变换 $sigma ( epsilon.alt_i )$ 将向量 $epsilon.alt_i$ 移动到同一空间另一位置. 我们可以先描述两个位置下的基底变换, 然后将原坐标套用在新基底下得到变换后的向量.

对于矩阵而言, 初始的旧基底即 $I$ 描述的标准坐标系基底. 线性变换总表示为左乘.


$
B = P_(H arrow.l E) A P_(E arrow.l H)
$


#line(length: 100%)

== 例题

两组基底: 
$
epsilon.alt_1 \, epsilon.alt_2 \, epsilon.alt_3
$
 
$
eta_1 \, eta_2 \, eta_3
$
 定义线性变换 $sigma$: 
$
sigma ( epsilon.alt_i ) = eta_i \, med i = 1 \, 2 \, 3
$


1. 写出由基 ${epsilon_(i)}$ 到 ${eta_(i)}$ 的过渡矩阵.
2. 求 $sigma$ 在基 ${epsilon_(i)}$ 下的矩阵
3. 求 $sigma$ 在基 ${eta_(i)}$ 下的矩阵
