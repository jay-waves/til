#import "@local/ypst-template:0.1.0": template, sidenote

#show: template

= 线性时不变系统

Linear Time-Invariant System（LTI），也称线性移不变系统（LSI）。

== 线性

对于系统 $T$，满足：

- 可加性：

  $ T[x_1 + x_2] = T[x_1] + T[x_2] $

- 一阶齐次性：

  $ T[a x_1] = a T[x_1] $

== 时不变性（移不变）

如 $T[x(n)] = y(n)$，则：

$ T[x(n - n_0)] = y(n - n_0) $

输入序列的移位等于输出序列移位，保持输出序列形态不变。


= 常见序列

== 单位抽样序列

离散时间单位抽样序列：

$
delta(n) = cases(
  1 & "if" n = 0,
  0 & "if" n != 0
)
$

$
x(m) delta(n-m) = cases(
  x(m) & "if" n = m,
  0 & "otherwise"
)
$

== 单位阶跃序列

$
u(n) = cases(
  1 & "if" n >= 0,
  0 & "if" n < 0
)
$

== 矩形序列

$
R_N (n) = cases(
  1 & "if" 0 <= n <= N-1,
  0 & "otherwise"
)
$


= 卷积

对于离散时间 LTI 系统：

$
y(n)
  &= x(n) * h(n) \
  &= sum_(m=-infinity)^infinity x(m) h(n-m) \
  &= sum_(m=-infinity)^infinity x(n-m) h(m)
$

$h(n)$ 表示 LTI 系统的冲激响应，*卷积*代表输入信号 $x(n)$ 经过系统 $h(n)$ 得到的输出结果。

卷积满足交换律：

$
x(n) * h(n) = h(n) * x(n)
$


== 相关

对于实值离散时间信号：

$
r_(x y)(n)
  &= sum_(m=-infinity)^infinity x(m) y(m-n) \
  &= x(n) * y(-n)
$

*相关*指两个信号之间的相互关系。

卷积满足交换律，互相关函数一般不满足交换律，但满足：

$
r_(x y)(n) = r_(y x)(-n)
$


== 卷积定理

*时域卷积等于频域乘积，时域乘积等于频域卷积。*

对于连续时间傅里叶变换：

$
cal(F)(f * g) = F(omega) G(omega)
$

$
cal(F)(f(t) g(t))
  = frac(1, 2 pi) (F * G)(omega)
$


