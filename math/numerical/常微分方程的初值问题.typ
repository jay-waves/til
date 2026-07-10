#import "../../appx/theme.typ": tufte, note

#show: tufte

#let bmat(..args) = $mat(delim: "[", ..args)$
#let vmat(..args) = $mat(delim: "|", ..args)$

== IVP 问题
给定常微分方程及其初值:

$ {y'(t) & = & f(t , y) , quad t in [ t_0 , T ]\
y(t_0) & = & y_0 $

目标是求满足上式的函数 $y(t)$. 方程的解析解为:

$ y(t) = y_0 + integral_(t_0)^t f ( s , y(s) ) thin d s $

当求解析解很困难时, 需要逼近该方程, 直接求某个位置 $t_i$ 上的数值解 $y(t_i)$.

=== 假设条件
要保证 "解存在且唯一" , 通常对右端 $f(t , y)$ 要求: 1. 连续性: $f(t , y)$ 在区域内连续, 保证解存在 2. 满足 Lipschitz 条件: 存在常数 $L > 0$, 使得任意 $y_1 , y_2$ 满足: $ | f(t , y_1) - f(t , y_2) | lt.eq L | y_1 - y_2 | $

== 单步迭代法
把区间 $[ t_0 , T ]$ 用步长 $h$ 划分: $t_n = t_0 + n h , quad n = 0 , 1 , dots.h$

已知近似值 $y_n approx y(t_n)$, 继续单步迭代 (单步, 指仅依赖当前点进行迭代):

$ y_(n + 1) = y_n + h times phi(t_n , y_n , h) $

其中 $phi$ 是某种斜率近似, 称为增量函数.

=== 单步迭代法的误差
定理: #strong[如果增量函数 $phi$ 对 $y$ 满足 Lipschitz 条件, 并且单步法的#emph[局部截断误差]是 $R_(n + 1) = O ( h^(p + 1) )$, 那么单步法的#emph[全局误差]为: $e_(n + 1) = O(h^p)$]

证明:

- 全局误差是: $e_n = y(t_n) - y_n$
- 局部截断误差: $R_(n + 1) = y ( t_(n + 1) ) - y(t_n) - h phi ( t_n , y(t_n) , h )$
- 迭代次数: $n approx frac(t_n - t, h)$

#quote(block: true)[
#link("http://oss.jay-waves.cn/til/单步迭代法的误差证明.png")[详细证明如图]
]

单步迭代法的全局误差为 $O(h^p)$, 那么就称其为 $p$ 阶方法.

== Runge--Kutta (RK) 方法
RK 法仍是单步迭代法, 但是每步估计”增量函数”的方法不同.

最一般的 s 阶 RK 公式:

$ cases(
  k_1 = f(t_n, y_n),
  k_2 = f(t_n + a_2 h, y_n + h b_21 k_1),
  dots.v,
  k_s = f(t_n + a_s h, y_n + h sum_(j = 1)^(s - 1) b_(s j) k_j),
  y_(n + 1) = y_n + h sum_(i = 1)^s c_i k_i,
  a_s = sum_(j = 1)^(s - 1) b_(s j),
) $

- 所有 $a_i , b_(i j) , c_i$ 都是待定系数.
- $k_i$ 代表不同位置的”试探斜率”

#strong[经典 RK 法的系数, 是和精确解泰勒展开对比得出的].

=== 欧拉法
欧拉法直接使用 $f(t_n , y_n)$ 作为增量函数:

$ y_(n + 1) = y_n + h f(t_n , y_n) $

欧拉法的全局误差为 $O(h)$, 稳定性较差.

=== 隐式欧拉法/后向欧拉法
$ y_(n + 1) = y_n + h f ( t_(n + 1) , y_(n + 1) ) $

即 $ y_(n + 1) - h f ( t_(n + 1) , y_(n + 1) ) = y_n $

适合刚性问题. 缺点是每一步都要求解一个非线性方程.

=== 修正欧拉法/显式梯形法
本质是用两点平均斜率代替单点斜率. 迭代公式为:

$ k_1 & = f(t_n , y_n) ,\
k_2 & = f(t_n + h , y_n + h k_1) ,\
y_(n + 1) & = y_n + h / 2 ( k_1 + k_2 ) $

全局误差为 $O(h^2)$

== 舍入误差
P199

== 稳定函数
模型方程: $y' = lambda y , lambda in bb(C)$

带入数值迭代法, 得到稳定函数: $y_(n + 1) = R(h lambda) y_n$

当 $| R(h lambda) | < 1$ 时, 数值方法每一步均衰减, 因此数值解是稳定的. (但精度不保证.)





