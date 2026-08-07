
#import "../appx/theme.typ": tufte, meta, note, theorem

#show: tufte

#meta(
  subtitle: [IMU-driven systems],
  revised: [2026-08-03],
  tags: ("robotics", "imu", "kinematics", "quaternion"),
)

#let bmat(..args) = math.mat(delim: "[", ..args)
#let vecb(x) = math.upright(math.bold(x))


= Inertial Measurement Unit (IMU)

IMU 提供加速度计（Accelerometer）和陀螺仪（Gyrometer），但仅用 IMU 进行定位，会导致随时间产生飘移。
直接由 IMU 积分计算出的姿态信息，称为刚体的*名义状态*（标称状态、Nominal State）。
通过融合 GPS 或视觉信息，可以在某程度上避免飘移，常用的技术是 ESKF。

The error-state Kalman fileter (ESKF) 负责估计名义状态附近的*小状态误差*。
最终真实的姿态表达为一个名义姿态和一个小误差之和。

#let r1 = (
  [Full state], [$vecb(x)_t$], [$vecb(x)$], [$delta vecb(x)$], [$vecb(x)_t = vecb(x) plus.o delta vecb(x)$], [], [],
)

#let r2 = (
  [Position], [$vecb(p)_t$], [$vecb(p)$], [$delta vecb(p)$], [$vecb(p)_t = vecb(p) + delta vecb(p)$], [], [],
)

#let r3 = (
  [Quaternion], [$vecb(q)_t$], [$vecb(q)$], [$delta vecb(q)$], [$vecb(q)_t = vecb(q) plus.o delta vecb(q)$], [], [],
)

#let r3 = (
  [Rotation Matrix], [$R_t$], [$R$], [$delta R$], [$R_t = R delta R$], [], [],
)

#let r5 = (
  [Accelerometer bias], [$vecb(a)_(b t)$], [$vecb(a)_b$], [$delta vecb(a)_b$], [$vecb(a)_(b t) = vecb(a)_b + delta vecb(a)_b$], [], [$vecb(a)_omega$],
)

#let r6 = (
  [Gyrometer bias], [$vecb(omega)_(b t)$], [$vecb(omega)_b$], [$delta omega vecb(omega)_b$], [$vecb(omega)_(b t) = vecb(omega)_b + delta vecb(omega)_b$], [], [$vecb(omega)_omega$],
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
  ..r5, 
  ..r6,
  ..r7, 
  ..r8,
  ..r9,
)

误差很小时，四元数误差可用三维旋转向量 $delta theta$ 表示：

#let ddelta = $delta theta$

$
delta vecb(q) = exp((ddelta) / 2)

= bmat( cos(norm(ddelta)\/2); ddelta/norm(ddelta) sin(norm(ddelta)\/2) )

approx bmat(1; 1/2 ddelta)
$


因此：

$ q_t = q times.o delta q,quad delta q tilde.eq bmat(1; 1/2 delta theta) $ 

定义真实状态 $x_t$ 、名义状态 $x$ 、误差状态 $delta x$ ，满足： 

$ 
x_t &= x plus.o delta x\ 
x &= bmat(p, v, q, a_b, omega_b, g),
$

IMU 输入为： 

$ u_m = bmat(a_m ; omega_m) $

传播时使用去除名义 bias 后的测量：

$ 
dot(p) & = v, \
dot(v) &= R(q) (a_m - a_b) + g, \
dot(q) &= 1/2 q times.o (omega_m - omega_b), \
$

bias 和重力的名义值通常认为在短时间内不变：

$
dot(a_b) & = 0,\
dot(omega_b) & = 0, \
dot(g) &= 0
$
