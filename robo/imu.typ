
#import "../appx/theme.typ": tufte, meta, note, theorem

#show: tufte

#meta(
  subtitle: [IMU-driven systems],
  revised: [2026-08-03],
  tags: ("robotics", "imu", "kinematics", "quaternion"),
)

#let bmat(..args) = math.mat(delim: "[", ..args)


= Inertial Measurement Unit (IMU)

IMU 提供加速度计（Accelerometer）和陀螺仪（Gyrometer），但仅用 IMU 进行定位，会导致随时间产生飘移。
通过融合 GPS 或视觉信息，可以在某程度上避免飘移。

The error-state Kalman fileter (ESKF) 

非线性传播标称状态（Nominal State），线性 KF 估计标称状态的微小误差。

定义真实状态 $x_t$ 、名义状态 $x$ 、误差状态 $delta x$ ，满足： 

$ 
x_t &= x plus.o delta x\ 
x &= bmat(p, v, q, a_b, omega_b, g),
$

