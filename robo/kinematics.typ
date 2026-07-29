
#import "../appx/theme.typ": tufte, meta, note, theorem

#show: tufte

#meta(
  subtitle: [机器人运动学],
  source: [Modern Robotics, Chapter 4-6],
  revised: [2026-07-13],
  tags: ("robotics", "modern-robotics"),
)

= Forward Kinematics

aaa

== D-H

Denavit-Hartenberg form:

$ T_04 = T_01 T_(12) T_23 T_34 $

== PoE

=== Space POE Formula
#image("../attach/robo-PoE.webp", width: 70%)

#let screw(i) = $[cal(S)_#i]$
#let mscrewm(i) = $M^(-1) [cal(S)_#i] M$
#let body(i) = $[cal(B)_#i]$


Set all joint variables to zero and record the home configuration $M=T(0)$.
Express every joint screw axis *in the fixed space frame* at this zero position. Then

$ T(theta) = e^([cal(S)_1] theta_1)
  e^([cal(S)_2] theta_2) dots
  e^([cal(S)_n] theta_n) M. $

The exponentials appear in joint order from base to end effector.  Although
$cal(S)_i$ is recorded at home, the preceding exponentials automatically move
its physical axis to the current configuration.

=== Body POE Formula

If $T$ carries a screw coordinate frame to the space frame, then

$ T e^([cal(S)]theta)T^(-1)
  =e^(T[cal(S)]T^(-1)theta)
  =e^(["Ad"_T cal(S)]theta). $

then,

$
T(theta)
  &= e^(screw(1)theta_1) dot ... dot e^(screw(n)theta_n) M \
  &= e^(screw(1)theta_1) dot ... dot M e^(mscrewm(n)theta_n) \
  &= e^(screw(1)theta_1) dot ... dot M e^(mscrewm(n-1)theta_(n-1))
     e^(mscrewm(n)theta_n) \
  &= M e^(body(1)theta_1) dot ... dot e^(body(n-1)theta_(n-1))
     e^(body(n)theta_n)
$

$body(i) = mscrewm(i)$, i.e., $cal(B)_i = ["Ad"_M^(-1)]cal(S)_i$

The ordering of the joint variables is unchanged; only the side on which the motion product acts
is different.

== the URDF Format

The URDF (Universal Robot Description Format) is an XML file sued by ROS2 to describe
 the kinematics, inertial properties, and link geometry of robots

=== Joints

#note[
*Joints* connect two links: a _parent_ and a _child_ link
  - types: prismatic, revolute, continuous (revolute without joint limits), fixed (virtual joints which doesnot permit any motion)
][
  在后续的运动学中，复杂关节都会被拆分为 1DoF Revolute/Prismatic 关节，这有利于计算。
]
  - _origin frame_: defines the child link frame relative to the parent link frame *in zero position*.
  - _axis_: unit vector along the rotation axis in child link frame.

```xml
<joint name="joint1" type="continuous">
  <parent link="base_link"/>
  <child link="link1"/>
  <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.089159"/>
  <axis xyz="0 0 1"/>
</joint>
```

=== Link

#note[
  The joints describe the kinematics of a robot, the link define tis mass properteis.
  - _origin frame_ describes the position and orientation of a frame at the link's center of mass relative to the link's joint frame
  - _inertia matrix_ ... (inertia matrix is symmetric, it's only necessary to define the terms on and above the diagonal)
][
  Inertia Matrix 详见机器人动力学（dynamics of robots）
]

#note[
```xml
<link name="world"/>

<link name="base_link"/>
  <inertial>
  <mass value="4.0"/>
  <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
  <inertia ixx="0.0044" ixy="0.0" ixz="0.0"
            iyy="0.044" iyz="0.0" izz="0.0072"/>
  </inertial>
</link>

<link name="link1"/>
  <inertial>
  <mass value="3.7"/>
  <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
  <inertia ixx="0.0102" ixy="0.0" ixz="0.0"
            iyy="0.0102" iyz="0.0" izz="0.0066"/>
  </inertial>
</link>
```
][
  URDF 不支持环结构
]

#note[
  = velocity kinematics & Statics
][
  Statics: 静力学；Dynamics：动力学
]

== Jocabion Matrix

$
bold(x) = bold(f)(bold(theta)),
quad
bold(x) = vec(x_1(t), dots.v, x_n(t)),
quad
bold(theta) = vec(theta_1(t), dots.v, theta_n(t))
$

对时间求导，由链式法则：

$
dot(x_i) = sum_(j=1)^n (partial f_i)/(partial theta_j) dot(theta_j)
$

写成矩阵形式：

$
vec( dot(x_1), dots.v, dot(x_n),)
=
mat(
  (partial f_1)/(partial theta_1), dots.h.c, (partial f_1)/(partial theta_n);
  dots.v, dots.down, dots.v;
  (partial f_n)/(partial theta_1), dots.h.c, (partial f_n)/(partial theta_n);)
vec( dot(theta_1), dots.v, dot(theta_n),)
$

定义 Jacobian：

$
J(bold(theta)) = (partial bold(x))/(partial bold(theta))
= mat(
  (partial f_1)/(partial theta_1), dots.h.c, (partial f_1)/(partial theta_n);
  dots.v, dots.down, dots.v;
  (partial f_n)/(partial theta_1), dots.h.c, (partial f_n)/(partial theta_n);
  )
$

因此：

$
  dot(bold(x)) = J(bold(theta)) dot(bold(theta))
$

== Space Jacobian of Vecolity Kniematics

#image("../attach/robot-2R.webp", width: 30%)

$
x_1 = L_1 cos(theta_1) + L_2 cos(theta_1 + theta_2) \
x_2 = L_1 sin(theta_1) + L_2 sin(theta_1 + theta_2)
$

对时间 $t$ 求导，得到：

$
vec( dot(x)_1, dot(x)_2) =
mat(
  -L_1 sin(theta_1) - L_2 sin(theta_1 + theta_2),quad, -L_2 sin(theta_1 + theta_2);
  L_1 cos(theta_1) + L_2 cos(theta_1 + theta_2),quad, L_2 cos(theta_1 + theta_2)
)
vec( dot(theta)_1, dot(theta)_2) = J(theta)dot(theta)
$


当且仅当 $det (J) = L_1 L_2 sin theta_2 = 0$ 时，某些方向的末端 速度无法继续由角速度产生。
该情况被称为*奇点 (Singularity)*。

== Statcis & Singularities

= Inverse Kinematics

For a n-DoF open chain with forward kinematics $T(theta)$ , $theta in RR^n$, the inverse
kinematics problem is: given a homogeneous transform $X \in S E(3)$, find solutions $theta$
that satisfy $T(theta) = X$.

== Numerical Newton-Raphson Method

#note[
  假设正运动学有定位公式 $x=f(theta)$, 目标位置为 $x_d$, 则逆运动学的误差定义为 $Delta x = x_d - f(theta_d)$。
  牛顿迭代法求解逆运动学方程是：

    $ x_d = f(theta_0) + J(theta_0) (theta_d - theta_0) = f(theta_0) + J(theta_0) Delta theta  $

  不能认为 $J(theta)_(m times n)$ 是可逆的，关节数 $n$ 通常会多于末端执行器维度 $m$。
][
  逆运动学不一定有解析解或没有简单形式的解析解，一般会用非线性方程的数值解法，
  详见 `../math/numerical/nonlinear-equations.typ`。

  这里 $J(theta)$ 是向量 $f in RR^m$ 对向量 $theta in RR^n$ 求导后的雅各比矩阵形式。

  $ J(theta) = (partial f) / (partial theta)(theta) = [(partial f_i)/(partial theta_j)]_(m times n) $
]


== Inverse Velocity Kinematics


= Closed-chain Kinematics

详见 @lynch2019 ，此处略

#bibliography("references.bib")
