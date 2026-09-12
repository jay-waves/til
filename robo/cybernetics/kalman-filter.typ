
#import "../../appx/theme.typ": tufte, note, theorem, definition

#show: tufte

= Kalman Filter 

卡尔曼滤波实际是一种*迭代*求解线性系统二次优化（最小方差估计）的方法。
在实际工程中，通常面临：系统建模不精确，传感器观测（修正）有噪声或偏差，很难估计系统当前的状态。
卡尔曼滤波提供了一种建模方法，在“建模”和“测量”之间做动态最优化融合。#footnote[一个常见的例子是 IMU 姿态传感器，详见 `../imu.typ`]

卡尔曼滤波始终在融合两个信息：建模预测值 $x_k^-$ ，传感器观测值 $z_k$ 

修正方式为：

$ x_k^+ = x_k^- + K_k (z_k - H x_k^-) $

其中 $K_i$ 是每次迭代的最优权重，被称为_卡尔曼增益（Kalman Gain）_。

#figure(
  image("../../assets/kalman-filter-wiki.png", width: 50%),
  caption: link("https://en.wikipedia.org/wiki/Kalman_filter")[Kalman Filter -- Wikipedia],
)

== 线性系统建模

系统的状态传播：（真实世界）

$
  x_k^- = F_k x_(k-1)^+ + B_k u_k + w_k 
$

系统的估计传播：（预测值）

$
  x_k^- = F x_(k-1)^+ + B u_k
$

#note[
- *$x_i^+$ 是观测后修正后结果，$x_i^-$ 是模型估计结果，$x_i$ 是实际值。*
- $F_k$ 是状态转移矩阵
- $B_k$ 是控制输入，$u_k$ 是控制向量。目前没有接入控制器，不关心。 
- $w_k$ 是状态转移过程的噪声，一般假设 $w_k ~ cal(N) (0, Q_k)$
][
  注意区分，$w$ 是状态转移噪声，$v$ 是传感器观察误差。
  KF 的迭代公式是固定的，但是 $F$ 建模比较困难。
]

== Kalman Gain 

设真实状态和建模预测之间的误差为：$ e^- = x - x^- $

每次测量时，传感器读数 $z$ 和真实状态间存在误差 $v$: $ v = H x - z $

其中
- $H$ 只是映射矩阵，将建模空间映射到测量空间
- $v ~ cal(N)(0, R)$ , 协方差矩阵 $"Cov"(v) = R$ 越大，传感器越不可信。

测量时，我们只能观测到带噪声的测量残差： $ r = z - H x^- = H(x - x^-) + v = H e^- + v $

#note[
  *问题变为，已知 $r$ ，如何通过增益 $K r$ ，逼近真实的误差 $e^-$ 。
  即让估计误差 $e^+ = e^- - K r$ 的方差尽可能小（二阶优化，最小方差估计）*。
][
  注意 $e^+$ 是向量，没有“最小”的定义，需要看它的统计量。
]

$ limits("min")_K E[norm(e^+)^2] = limits("min")_K E[norm(e^- - K r)^2] $

最小方差估计问题的最优性条件为（最优估计下，误差 $e^+$ 已经和 $r$ 没有线性相关性）： 

$ E[e^+ r^top] = 0 $

得到 $ E[(e^- - K r) r^top ] = E[e^- r^top] - K E[r r^top] = 0 $

因此 $ K = E[e^- r^top](E[r r^top])^(-1)  = "Cov"(e^-, r)"Cov"(r)^(-1) $

$"Cov"(e^-, r)$ 是状态误差和 $r$ 的相关性，展开可得（过程略）：

$ "Cov"(e^-, r) = E[e^- e^-^top] H^top = P^- H^top  $


$S="Cov"(r)$ 是 $r$ 本身的稳定性（可靠程度、置信度），展开可得：

$
  S 
    &= "Cov"(r) \
    &= H E[e^- e^-^top] H^top + "Cov"(v) \ 
    &= H P^- H^top + R
$

最终有：

$ K = P^- H^top (H P^- H^top + R)^(-1) $

== 迭代过程

一次迭代分为 Predict 和 Update 两个过程。

=== 预测阶段（Predict）

模型迭代（忽略控制项）：

$ x_k^- = F_k x_(k-1)^+ + w_k $

#let ek1 = $e_(k-1)$
#let xk1 = $x_(k-1)$
#let Mo(x, y: none) = {
  if y == none {
    $cal(M)[#x]$
  } else {
    $cal(M)[#x, #y]$ }
}

#note[

  不确定性传播 (注意 $"Cov"(e^+, w) = 0$）：

  $ 
    P_k^- 
      & = Mo(e_k^-) = Mo(x_k - x_k^-) \
      & = Mo(F xk1 + w - F xk1^+)\
      & = Mo(F ek1^+ + w_k) \
      & = F Mo(ek1) F^top + Mo(w) \
      & = F_k P^+_(k-1) F_k^top + Q_k 
  $

][
  这里定义二阶矩符号 $cal(M)(x) eq.def E[x x^top]$ ，

]

得到 $(x_k^-, P_k^-)$ ，接下来通过观测值进行修正。

=== 修正阶段（Update）

首先修正状态 #footnote[由于不涉及迭代，这里忽略掉迭代下表 $k$ ，即 $x_k$ 记为 $x$。] ：

$
  x^+ 
  &= x^- + K r \
  &= x^- + K (z - H x^-) 
$

修正协方差（不确定性），注意 $"Cov"(e^-, v) = 0$：

$
  P^+ 
    & = Mo(e^+) \
    & = Mo((I-K H)e^- - K v) \
    & = (I-K H) Mo(e^-) (I-K H)^top + K Mo(v) K^top \
    & = (I - K H) P^- (I- K H)^top + K R K^top 
$

此公式称为Joseph Form。

#quote[
还有一种简洁形式。可以证明：

$
  P^+
    & = (I - K H) P^- (I- K H)^top + K R K^top \
    & = (I - K H) P^-
$

在有限精度浮点数迭代计算中，该形式不具备数值稳定性。（暂且搁置）
]

最终，得到 $(x^+_k, P_k^+)$ 后，即可进入下一轮。

= 一维卡尔曼滤波

aaa
