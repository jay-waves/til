
#import "../../appx/theme.typ": tufte, note, theorem, definition, equate-lines

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

$
  x_k = A_k x_(k-1) + B_k u_k + w_k 
$

#note[
- $A_k$ 是状态转移矩阵
- $B_k$ 是控制输入，$u_k$ 是控制向量。目前没有接入控制器，不关心。 
- $w_k$ 是状态转移过程的噪声，一般假设 $w_k ~ cal(N) (0, Q_k)$
][
  注意区分，$w$ 是状态转移噪声，$v$ 是传感器观察误差。
]

== Kalman Gain 

设真实状态和建模预测之间的误差为：$ e^- = x_t - x^- $

每次测量时，传感器读数 $z$ 和真实状态间存在误差 $v$: $ v = H x_t - z $

其中
- $H$ 只是映射矩阵，将建模空间映射到测量空间
- $v ~ cal(N)(0, R)$ , 协方差矩阵 $"Cov"(v) = R$ 越大，传感器越不可信。

测量时，我们只能观测到带噪声的测量残差： $ r = z - H x^- = H(x_t - x^-) + v = H e^- + v $

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
S &= "Cov"(r) \
&= H E[e^- e^-^top] H^top + "Cov"(v) \ 
&= H P^- H^top + R
$

最终有：

$ K = P^- H^top (H P^- H^top + R)^(-1) $

== 迭代过程

= 一维卡尔曼滤波

