
#import "../appx/theme.typ": tufte, note, theorem, definition, equate-lines

#show: tufte

#set document(
  title: [IMU Driven Systems],
  date: datetime.today(),
  keywords: ("robotics", "imu", "kinematics", "quaternion")
)

#let bmat(..args) = math.mat(delim: "[", ..args)
#let vecb(x) = math.upright(math.bold(x))

= Inertial Measurement Unit (IMU)

IMU 集成了加速度计（Accelerometer）和陀螺仪（Gyrometer），提供刚体 (IMU) 坐标系下测量的加速度 $a_m$ 和角速度 $omega_m$。

直接由 IMU 积分计算出的姿态信息，称为刚体的*名义状态*（标称状态、Nominal State），随时间会产生误差（飘移），需要修正。
通过融合其他传感器信息（如 GPS 或视觉），可以减少和修正飘移。常见技术有 ESKF 或 Factor Graph。

#let r1 = (
  [Full state], [$vecb(x)_t$], [$vecb(x)$], [$delta vecb(x)$], [$vecb(x)_t = vecb(x) plus.o delta vecb(x)$], [], [],
)

#let r2 = (
  [Position], [$vecb(p)_t$], [$vecb(p)$], [$delta vecb(p)$], [$vecb(p)_t = vecb(p) + delta vecb(p)$], [], [],
)

#let r3 = (
  [Quaternion], [$vecb(q)_t$], [$vecb(q)$], [$delta vecb(q)$], [$vecb(q)_t = vecb(q) plus.o delta vecb(q)$], [], [],
)

#let r4 = (
  [Rotation Matrix], [$R_t$], [$R$], [$delta R$], [$R_t = R delta R$], [], [],
)

#let r5 = (
  [Accelerometer bias], [$vecb(a)_(b t)$], [$vecb(a)_b$], [$delta vecb(a)_b$], [$vecb(a)_(b t) = vecb(a)_b + delta vecb(a)_b$], [], [$vecb(a)_omega$],
)

#let r6 = (
  [Gyrometer bias], [$vecb(omega)_(b t)$], [$vecb(omega)_b$], [$delta vecb(omega)_b$], [$vecb(omega)_(b t) = vecb(omega)_b + delta vecb(omega)_b$], [], [$vecb(omega)_omega$],
)

#let r7 = (
  [Gravity vector], [$vecb(g)_(b t)$], [$vecb(g)_b$], [$delta vecb(g)_b$], [$vecb(g)_(b t) = vecb(g)_b + delta vecb(g)_b$], [], [],
)

#let r8 = (
  [ Acceleration], [$vecb(a)_t$], [], [], [], [$vecb(a)_m$], [$vecb(a)_n$],
)

#let r9 = (
  [Angular rate], [$vecb(omega)_t$], [], [], [], [$vecb(omega)_m$], [$vecb(omega)_n$],
)

#table(
  columns: 7,
  table.header(
    [Magitude], [True], [Nominal], [Error], [Composition], [Measured], [Noise],
  ),
  ..r1,
  ..r2,
  ..r3,
  ..r4,
  ..r5, 
  ..r6,
  ..r7, 
  ..r8,
  ..r9,
)

== 典型的 IMU 测量原理

陀螺仪：

$ omega_m = omega_t + omega_"bt" + omega_n $

- $omega_t$ 是真实角速度
- $omega_"bt"$ 是陀螺仪的物理偏差 (bias)，是由于器件不理想导致的零位偏置 (offset)，是持续存在可估计的系统误差。
  由于该值持续存在，会导致积分计算位姿 $T = (p, R)$ 有随时间线性增长的*偏移（drift）*。
- $omega_n$ 是陀螺仪的测量噪声误差 (noise)，在每个采样时刻快速变化，不可直接估计的随机扰动。
- $omega_m$ 是 IMU 实际测量的角速度

同理，加速度：

$ a_m = a_t + a_"bt" + a_n $

注意，这里用了 "bt" 而不是 "b"，因为滤波器对 bias 的估计（名义值）也不是准确的 ：

$ a_"bt" = a_b plus.o delta a $ <eq:bias>

#note[
注意，bias 并不是常量，仍然会随时间缓慢随机改变，因此建模为白噪声 $a_w$ 驱动的随机游走：

$ dot(a_"bt") = a_w,quad E[a_w] = 0 $

作为对比，噪声误差 $a_n$ 本身是高斯分布的，随着时间的累计基本为零。

$ a_n ~ cal(N)(0, sum_n),quad E[a_n] = 0 $
][
可以理解为：bias 是持续存在的偏置量，并且随时间缓慢移动；noise 是快速变化的噪声，和时间无关。
]

=== 姿态表达 (orientation)

IMU 的测量值是相对刚体坐标系的，因此旋转量 $R$ 表达从刚体坐标系 $BB$ 到
世界坐标系 $WW$ 的姿态变换（orientation）。

旋转可以用旋转矩阵 $R$ 或四元数 $q$ 表示，两者是完全等价的。#footnote[详情参考刚体力学：`./rigid-bodies.typ`]
- 测量值等价 $ q_m <==> R_m $
- 误差等价（$delta theta$ 是一个微小旋转） $ delta q = exp(delta theta \/ 2) <==> delta R = exp([delta theta]_times) $
- 真实值等价 $ q_t = q times.o delta q <==> R_t = R delta R $

默认情况下，$w, v, p, g$ 等物理量都在 $WW$ 世界坐标系中表示，有加速度公式：

$ dot(v) = R a_t + g = R(a_m - a_b - a_n) + g $

其中 $a_"xxx"$  是 IMU 在 $BB$ 刚体坐标系的测量量，通过 $R$ 旋转到世界坐标系。

=== 四元数处理

误差很小时，四元数误差可用三维旋转向量 $Delta theta$ 表示：

#let ddelta = $Delta theta$

$
delta vecb(q) = exp((ddelta) / 2)

= bmat( cos(norm(ddelta)\/2); ddelta/norm(ddelta) sin(norm(ddelta)\/2) )

approx bmat(1; 1/2 ddelta)
$


因此：

$ q(t+upright(d) t) = q(t) times.o delta q,quad delta q tilde.eq bmat(1; 1/2 ddelta) $ 

由于 $ddelta = w "d"t$, 得到：

$ q(t + upright(d)t) = q(t) times.o (bmat(1; 0) + bmat(0; 1/2 ddelta)) =q(t) + q(t) times.o 1/2 bmat(0; omega) upright(d)t  $

于是：

$ dot(q(t)) = (q(t+ upright(d)t) - q(t))/ (upright(d)t) = 1/2 q(t) times.o bmat(0; omega) $

=== 微分 

旋转的累积：

$ R_(k+1) = R_k e^([w_k "d"t]) approx R_k (I + [w_k "d"t]) $

$ q_(k+1) = q_k times.o delta q = q_k times.o bmat(cos(theta/2); (w_k "d"t)/theta sin(theta/2)) $

注意，因为 $ddelta = w "d"t$，可知，

$ R(t + "d"t) = R(t) e^([ddelta]) = R(t) e^([w]"d"t) approx R(t)(I + [w]"d"t) $

$ dot(R) = (R(t + "d"t) - R(t))/("d"t) approx R(t) [w] $ <eq:dotR>

位置与速度的累计：

$ p_(k+1) = p_k + v "d"t + 1/2 (R(a_m - a_n - a_b) + g)"d"t^2 $

$ v_(k+1) = v_k + (R(a_m - a_n - a_b) + g) "d"t $

== 名义状态建模

IMU 状态建模：（$g$ 用于对齐世界坐标系，除此外的其他量均位于 $BB$ 体坐标系）

$
 x = bmat(p, v, q, a_b, w_b, g) 
$

IMU 输入为： 

$ u_m = bmat(a_m ; w_m) $

重力加速度 $g$ 被认为短时间不变，初始时，将世界坐标系的 $z$ 轴和 $g$ 方向对齐。

#definition[Nominal State：

#equate-lines($
&dot(p) && = v, \
&dot(v) &&= R(q) (a_m - a_b - a_n) + g, \
&dot(q) &&= 1/2 q times.o (omega_m - omega_b - omega_n), \
&dot(a)_"bt" && = a_w, \
&dot(omega)_"bt" && = omega_w, \
&dot(g) &&= 0
$)
]

== 误差状态建模

上文 @eq:bias 提到，$a_b, w_b$ 均为名义值，需要不断误差修正：

#definition[Error State: #footnote[详细推导见 @sola2017 P58-60, ESKF 的标准卡尔曼滤波形式见 P61]

#equate-lines($
&dot(delta p) && = delta v, \
&dot(delta v) &&= -R [a_m - a_b] delta theta - R delta a_b - R a_n + delta g, #<ESKF1>\
&dot(delta theta) &&= -[w_m - w_b]delta theta  - delta w_b - w_n, #<ESKF2> \
&dot(delta a)_b && = a_w, #<ESKF3> \
&dot(delta omega)_b && = omega_w, #<ESKF4> \
&dot(delta g) &&= 0
$)

]

#pagebreak()

=== @ESKF1

不妨令 $a = a_m - a_b$，有：

$ dot(v)_t = R_t (a - delta a_b - a_n) + g + delta g,quad R_t approx R(I+[delta theta]) $

忽略二阶无穷小量：

$ dot(v)_t approx R a + R[delta theta]a- R delta a_b - R a_n + g + delta g $

两侧约掉名义量 $dot(v) = R(a) + g$，得到：

$ 
dot(delta v) &= R[delta theta]a- R delta a_b - R a_n + delta g \ 
& = -R[a]delta theta - R delta a_b  - R a_n + delta g
$

#linebreak()

上式等价于：

$
dot(delta v) &= delta (R a) + delta g \ 
&= - R[a]delta theta + R delta a + delta g, \
delta a &= -delta a_b - a_n
$

=== @ESKF2

将 $R_t approx R(I + [delta theta])$ 同乘 $R^top$ ，得到：

$ Delta R = R^top R_t approx I + [delta theta] $ <eq:DR>

不妨令 $w = w_m - w_b$。因为 @eq:dotR ，得到：

$
dot(R^top) &= -[w]R^top \
dot(R_t) &= R_t [w_t] = R_t [w - delta w_b - w_n] 
$

代入 @eq:DR 中，得到：

$ 
dot(delta theta) = -[w] delta theta + delta w, quad delta w = - delta w_b - w_n
$

=== @ESKF3 & @ESKF4 

偏置（bias）是一个缓慢的随机游走：（固定零点偏置，逐帧累计白噪声）

$ dot(a_"bt") = a_w $

偏置的名义值（nominal bias）保持不变（我觉得等价于固定零偏，没有累计噪声）

$ dot(a_b) = 0 $

因为 $a_"bt" = a_b + delta a_b$ ，两侧求导可知：

$ a_w = dot(delta a_b) $

== ESKF (fusing IMU with sensors)

#note[
- VIO: Camera + IMU + Slided Windows 
- LIO: LiDAR + IMU + ESIKF (IEKF) 。LiDAR 是激光雷达。
- GNSS + IMU 
- Radar + IMU。Radar 是无线电波雷达。
][
  Radar 发射的无线电波、毫米波，比光的频率低很多，容易测量。因此，可以利用多普勒效应来测量径向速度
]

=== Kalman Filter 

将上述 Error-State 的建模，改写为标准卡尔曼滤波形式（连续时间下）：

$
delta x = bmat(
  delta p;
  delta v;
  delta theta;
  delta a_b;
  delta omega_b;
  delta g
),
quad
n = bmat(
  a_n;
  omega_n;
  a_w;
  omega_w
)
$

$
dot(delta x) = F delta x + G n
$

误差状态转移矩阵：

$
F = bmat(
  0, I, 0, 0, 0, 0;
  0, 0, -R [a], -R, 0, I;
  0, 0, -[w], 0, -I, 0;
  0, 0, 0, 0, 0, 0;
  0, 0, 0, 0, 0, 0;
  0, 0, 0, 0, 0, 0
)
$

噪声输入矩阵：

$
G = bmat(
  0, 0, 0, 0;
  -R, 0, 0, 0;
  0, -I, 0, 0;
  0, 0, I, 0;
  0, 0, 0, I;
  0, 0, 0, 0
)
$

如果噪声满足：

$
E(n(t) n(tau)^T) = Q_c delta(t - tau),
$

那么连续时间的协方差传播：

$
dot(P) = F P + P F^T + G Q_c G^T
$

=== reset 

=== 离散化（离散采样）

TODO 

== Factor Graph 



#bibliography("./references.bib")
