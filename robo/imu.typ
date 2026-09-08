
#import "../appx/theme.typ": tufte, meta, note, theorem, definition, equate-lines

#show: tufte

#meta(
  subtitle: [IMU-driven systems],
  revised: [2026-08-03],
  tags: ("robotics", "imu", "kinematics", "quaternion"),
)

#let bmat(..args) = math.mat(delim: "[", ..args)
#let vecb(x) = math.upright(math.bold(x))


= Inertial Measurement Unit (IMU)

IMU 集成了加速度计（Accelerometer）和陀螺仪（Gyrometer），提供刚体 (IMU) 坐标系下测量的加速度 $a_m$ 和角速度 $omega_m$。
直接由 IMU 积分计算出的姿态信息，称为刚体的*名义状态*（标称状态、Nominal State），随时间会产生误差（飘移），需要修正。
通过融合其他传感器信息（如 GPS 或视觉），可以减少和修正飘移。常见技术有 ESKF 或 Factor Graph。

The error-state Kalman filter (ESKF) 负责估计名义状态附近的*小状态误差*。
最终真实的姿态表达为一个名义姿态和一个小误差补偿之和。

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

== 典型的测量模型

陀螺仪：

$ omega_m = omega_t + omega_"bt" + omega_n $

- $omega_t$ 是真实角速度
- $omega_"bt"$ 是陀螺仪的偏移误差 (bias) ，是持续存在，可估计的系统误差。
- $omega_n$ 是陀螺仪的测量白噪声 (noise)，在每个采样时刻快速变化，不可直接估计的随机扰动。
- $omega_m$ 是 IMU 实际测量的角速度

同理，加速度：

$ a_m = a_t + a_"bt" + a_n $


== 姿态表达 (orientation)

IMU 的测量值是相对刚体坐标系的，因此旋转量 $R$ 表达从刚体坐标系 $BB$ 到
世界坐标系 $WW$ 的姿态变换（orientation）。

旋转可以用旋转矩阵 $R$ 或四元数 $q$ 表示，两者是完全等价的。#footnote[详情参考刚体力学：`./rigid-bodies.typ`]
- 测量值等价 $ q_m <==> R_m $
- 误差等价（$delta theta$ 是一个微小旋转） $ delta q = exp(delta theta \/ 2) <==> delta R = exp([delta theta]_times) $
- 真实值等价 $ q_t = q times.o delta q <==> R_t = R delta R $

默认情况下，$w, v, p, g$ 等物理量都在 $WW$ 世界坐标系中表示，有加速度公式：

$ dot(v) = R a_t + g = R(a_m - a_b - a_n) + g $

其中 $a_"xxx"$  是 IMU 在 $BB$ 刚体坐标系的测量量，通过 $R$ 旋转到世界坐标系。

== 四元数处理

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

== 微分 

旋转的累积：

$ R_(k+1) = R_k exp([w_k "d"t]) approx R_k (I + [w_k "d"t]) $

$ q_(k+1) = q_k times.o delta q = q_k times.o bmat(cos(theta/2); (w_k "d"t)/theta sin(theta/2)) $

注意，因为 $ddelta = w "d"t$，可知，

$ R(t + "d"t) = R(t) exp([ddelta]) = R(t) exp([w]"d"t) approx R(t)(I + [w]"d"t) $

$ dot(R) = (R(t + "d"t) - R(t))/("d"t) approx R(t) [w] $

位置与速度的累计：

$ p_(k+1) = p_k + v Delta t + 1/2 (R(a_m - a_n - a_b) + g)Delta t^2 $

$ v_(k+1) = v_k + (R(a_m - a_n - a_b) + g) Delta t $

== 真实状态建模

由于误差（bias）是长期积累的，时间相关的，因此也被记入状态中。噪声（noise）是随机扰动，不计入状态。
$g$ 是一个未知量（指精确值），初始的世界坐标系和重力方向不一定严格对齐，因此也是需要估计和修正的。

$
 x = bmat(p, v, q, a_b, omega_b, g) 
$

// 定义真实状态 $x_t$ 、名义状态 $x$ 、误差状态 $delta x$ ，满足： 
//
// $ 
// x_t &= x plus.o delta x\ 
// x &= bmat(p, v, q, a_b, omega_b, g),
// $
//
IMU 输入为： 

$ u_m = bmat(a_m ; omega_m) $

IMU 积累的误差（bias）为 $a_b, omega_b$，误差积累的过程建模为随机游走，由白噪声 $a_w, omega_w$ 驱动。
重力加速度 $g$ 被认为短时间不变，初始时，将世界坐标系的 $z$ 轴和 $g$ 方向对齐。

#definition[
IMU 姿态建模 (nominal state)：


#equate-lines($
dot(p) & = v, \
dot(v) &= R(q) (a_m - a_b - a_n) + g, \
dot(q) &= 1/2 q times.o (omega_m - omega_b - omega_n), \
dot(a)_b & = a_w, \
dot(omega)_b & = omega_w, \
dot(g) &= 0
$)
]

== 误差状态建模

$delta a_b$ 是对 $a_b$ 的修正，即，对偏移量的小修正。当 IMU 数据更新时，通过 $delta a_b$ 来修正 $a_b$ 估计。

#definition[
Error-state: #footnote[详细推导见 @sola2017 P58-60, ESKF 的标准卡尔曼滤波形式见 P61]
#equate-lines($
dot(delta p) & = delta v, \
dot(delta v) &= -R [a_m - a_b] delta theta - R delta a_b - R a_n + delta g, #<ESKF1>\
dot(delta theta) &= -[w_m - w_b]delta theta  - delta w_b - w_n, \
dot(delta a)_b & = a_w, \
dot(delta omega)_b & = omega_w, \
dot(delta g) &= 0
$)
]

@ESKF1

=== $delta v$ 推导

#theorem[
  aaaa
]

=== $delta theta$ 推导 

== ESKF (fusing IMU with sensors)


#note[
- VIO: Camera + IMU + Slided Windows 
- LIO: LiDAR + IMU + ESIKF (IEKF) 。LiDAR 是激光雷达。
- GNSS + IMU 
- Radar + IMU。Radar 是无线电波雷达。
][
  Radar 发射的无线电波、毫米波，比光的频率低很多，容易测量。因此，可以利用多普勒效应来测量径向速度
]

== Factor Graph 



#bibliography("./references.bib")
