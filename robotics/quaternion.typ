#import "../.vscode/tufte.typ": tufte, note

#show: body => tufte(
  title: [四元数],
  subtitle: [],
  body,
)

#note[
  来源：#link("https://github.com/Krasjet/quaternion")[四元数于三维旋转]，Krasjet，CC BY-NC-SA 4.0。原文中的远程图片在这里保留为链接。
]

= 复数

复数 $a + b i$ 同构映射于矩阵形式：

$
mat(delim: "[", a, -b; b, a)
$

如：

$
i arrow.l.r mat(delim: "(", 0, -1; 1, 0)
$

记复平面 $z = a + b i$ 与 $Re$ 的正方向夹角为 $theta$，那么：

$
cos theta = a / sqrt(a^2 + b^2)
$

$
sin theta = b / sqrt(a^2 + b^2)
$

记 $abs(z) = sqrt(a^2 + b^2)$，复数的同构矩阵可表示为_缩放矩阵_与_旋转矩阵_的乘积：

$
z = abs(z) dot I dot mat(delim: "[", cos theta, -sin theta; sin theta, cos theta)
  = mat(delim: "[", abs(z), 0; 0, abs(z))
    dot mat(delim: "[", cos theta, -sin theta; sin theta, cos theta)
  = abs(z) e^(i theta)
$

因此，一个复数可以表达一次旋转与缩放的线性变换。

= 轴角式旋转

设经过原点的旋转轴 $bold(u) = (x, y, z)^T$ 满足 $abs(bold(u)) = 1$，给定向量 $bold(v)$，使其沿着旋转轴转动 $theta$ 角度，得到 $bold(v)'$。这里使用右手系统来定义旋转正方向。

轴角旋转有三个#link("rigid-bodies.md")[自由度]：一个表示 $theta$，两个表示单位向量 $bold(u)$。

#link("http://oss.jay-waves.cn/til/20260331231138727.avif")[轴角旋转示意图]

将向量 $bold(v)$ 沿垂直于轴 $bold(u)$ 和平行于轴的两个方向分解为 $bold(v)_1, bold(v)_2$。平行分量 $bold(v)_1$ 在旋转后保持不变，垂直分量旋转后变为：

$
bold(v)'_2 = cos theta dot bold(v)_2 + sin theta dot (bold(u) times bold(v)_2)
$

由于：

$
bold(v)_2 times bold(u) = bold(v) times bold(u)
$

$
bold(v)_1 = (bold(u) dot bold(v)) bold(u), quad
bold(v)_2 = bold(v) - bold(v)_1
$

因此，旋转后的向量有_三维旋转公式_：

$
bold(v)' =
  cos theta dot bold(v)
  + (1 - cos theta)(bold(u) dot bold(v)) bold(u)
  + sin theta (bold(u) times bold(v))
$

= 四元数

定义四元数 $q in bb(H)$ 满足：

$
q = a + b i + c j + d k = [a, bold(v)]
$

其中：

$
a, b, c, d in bb(R), quad
i^2 = j^2 = k^2 = i j k = -1, quad
bold(v) = (b, c, d)^T in bb(R)^3
$

四元数的模长：

$
abs(q) = sqrt(a^2 + b^2 + c^2 + d^2)
$

纯四元数：

$
q = [0, bold(x)]
$

四元数的共轭：

$
overline(q) = [s, -bold(v)]
$

四元数的互乘不满足交换律，因为叉乘不满足交换律：

$
q_1 q_2 - q_2 q_1 = [0, 2 bold(x) times bold(y)]
$

换言之，当 $bold(x) parallel bold(y)$ 时，满足：

$
q_1 q_2 = q_2 q_1
$

== Grassmann Product

$
q_1 q_2 =
  (a e - (b f + c g + d h))
  + (b e + a f + c h - d g) i
  + (c e + a g + d f - b h) j
  + (d e + a h + b g - c f) k
$

令向量 $bold(x) = (b, c, d)^T$，$bold(y) = (f, g, h)^T$，那么：

$
q_1 = [a, bold(x)], quad q_2 = [e, bold(y)]
$

$
q_1 q_2 =
  [a e - bold(x) dot bold(y),
   e bold(x) + a bold(y) + bold(x) times bold(y)]
$

$
q overline(q) = overline(q) q = [a^2 + bold(v)^2, 0]
$

四元数的逆：

$
q^(-1) q = 1, quad
q^(-1) = overline(q) / abs(q)^2
$

纯四元数的积：

$
v u = [0, bold(v)] dot [0, bold(u)]
  = [-bold(v) dot bold(u), bold(v) times bold(u)]
$

== 代数结构

四元数空间是一个#link("../math/linalg/向量分析/线性变换.md")[线性空间]，满足：

$
bb(H) equiv bb(R) + bb(R)^3 equiv bb(R)^4
$

四元数空间的代数结构是#link("../math/algebra/环/环.md")[除环]，满足：

1. 加法是交换群
2. 乘法封闭，结合律
3. 乘法单位元
4. 分配律
5. 非零元素有逆元

== 单位四元数

任意单位四元数都可表示为：

$
q = cos theta + sin theta bold(u)
  = [cos theta, sin theta bold(u)]
$

其中 $bold(u)$ 是单位旋转轴向量。

$
abs(q) = 1, quad q^(-1) = overline(q)
$

也可以证明：

$
q^2 = q q = [cos(2 theta), sin(2 theta) bold(u)]
$

= 旋转与单位四元数

设垂直于轴的四元数 $v_2 = [0, bold(v)_2]$，垂直分量的旋转公式可以改写为：

$
v'_2 = q v_2
     = [cos theta, sin theta bold(u)] [0, bold(v)_2]
     = [-sin theta bold(u) dot bold(v)_2,
        cos theta bold(v)_2 + sin theta (bold(u) times bold(v)_2)]
     = [0, bold(v)'_2]
$

令 $bold(w) = cos theta bold(v)_2 + sin theta (bold(u) times bold(v)_2)$，右乘 $q^(-1)$ 得到：

$
(q v_2) q^(-1)
  = [0, bold(w)] [cos theta, -sin theta bold(u)]
  = [0, cos(2 theta) bold(v)_2 + sin(2 theta)(bold(u) times bold(v)_2)]
$

对于平行分量 $v_1$ 有：

$
q v_1 = [cos theta, sin theta bold(u)] [0, (bold(u) dot bold(v)) bold(u)]
$

$
(q v_1) q^(-1) = [0, bold(v)_1]
$

令 $phi = 2 theta$，于是_四元数形式的三维旋转公式为_：

$
q v q^(-1)
  = [0,
     cos phi bold(v)
     + (1 - cos phi)(bold(u) dot bold(v)) bold(u)
     + sin phi (bold(u) times bold(v))]
$

== 三维旋转公式

任意向量 $bold(v)$ 绕单位轴向量 $bold(u)$ 旋转 $phi$ 度后得到 $bold(v)'$，在四元数定义下有旋转公式：

$
v' = q v q^(-1) = q v overline(q)
$

其中：

- $q = [cos(phi / 2), sin(phi / 2) bold(u)]$
- $v = [0, bold(v)]$

公式变形：

$
v' = q (v_1 + v_2) q^(-1) = v_1 + q v_2 q^(-1)
$

可以证明 $q v_2 = v_2 q^(-1)$，因此有：

$
v' = v_1 + q^2 v_2
   = v_1 + [cos phi, sin phi bold(u)] v_2
$

== 旋转的复合

设 $q_1, q_2$ 都是单位旋转四元数，可以证明 $q_1^(-1) q_2^(-1) = (q_2 q_1)^(-1)$，因此复合旋转：

$
v'' = q_2 q_1 v q_1^(-1) q_2^(-1)
    = (q_2 q_1) v (q_2 q_1)^(-1)
$

向量绕 $q$ 旋转 $theta$，就等于反方向旋转 $2 pi - theta$，用 $-q$ 表示：

$
(-q) v (-q)^(-1) = q v q^(-1)
$

为了走“最短路径”，会先求夹角，取 $pi$ 内旋转。

每一个固定旋转轴的单位四元数，可以生成一个子群。这个子群与复指数 $e^(i theta) = cos theta + i sin theta$ 是同构的。#footnote[复指数相关知识见 #link("../math/calculus/复数.md")[复数]、#link("../math/calculus/三角函数.md")[三角函数]。] 复指数表示一个平面的旋转。

$
upright("Sp")(1) equiv upright("SU")(2)
$

= 插值

在工程上，*单位旋转四元数用于表示一个刚体的旋转姿态（Orientation）*。刚体运动过程中，$q_0$ 表示初始位姿，$q_1$ 表示目标位姿，两者是离散的。需要用插值，来获取中间状态，完成平滑旋转过渡。

设一个旋转 $Delta q$，那么有：

$
Delta q dot q_0 = q_1, quad
Delta q = q_1 q_0^(-1)
$

$
q_t = (Delta q)^t q_0
$

注意，由于 $abs(q) = 1$，四元数实际上只有三个自由度。旋转姿态活动于一个超球面内，但两个姿态 $q_0, q_1$ 位于同一个圆内，因此 $Delta q$ 只有两个自由度。

#link("http://oss.jay-waves.cn/til/orientation_lerp.avif")[Orientation lerp 示意图]

== Slerp 插值

#link("http://oss.jay-waves.cn/til/slerp.avif")[Slerp 示意图]

_球面线性插值（Spherical Linear Interpolation）_：

$
bold(v)_t = alpha bold(v)_0 + beta bold(v)_1
$

同乘 $bold(v)_0$，由于单位向量性质，得到：

$
bold(v)_0 bold(v)_t =
  alpha (bold(v)_0 bold(v)_0) + beta (bold(v)_0 bold(v)_1)
$

$
cos(t theta) = alpha + beta cos theta
$

同理，同乘 $bold(v)_1$，得到：

$
cos((1 - t) theta) = alpha cos theta + beta
$

解方程，得到 $upright("Slerp")(q_0, q_1, t)$：

$
beta = sin(t theta) / sin theta, quad
alpha = sin((1 - t) theta) / sin theta, quad
theta = cos^(-1)(q_0 q_1)
$

== Squad 插值

Slerp 在两点间对角度进行线性插值，但不能保证端点处*平滑*。假设三个姿态 $q_0, q_1, q_2$，那么 $upright("Slerp")(q_0, q_1)$ 和 $upright("Slerp")(q_1, q_2)$ 在 $q_1$ 点不能保证导数连续。

Squad 算法：

$
upright("Quad")(q_0, q_1, q_2, q_3; t)
  = upright("Slerp")(upright("Slerp")(q_0, q_3; t), upright("Slerp")(q_1, q_2); 2 t (1 - t))
$

$h(t) = 2 t (1 - t)$ 是一个对称的抛物线权重函数。

= 四元数与李群

2D、3D 旋转矩阵都是#link("../math/linalg/对称矩阵.md")[_正交矩阵（Orthogonal Matrix）_]，它们的行列式值为 $1$，代表着旋转；$-1$ 代表反射。

李群（Special Orthogonal Group, SO）是讨论旋转的更一般情况。单位四元数同构于某种李群。

= 参考资料

#link("https://github.com/Krasjet/quaternion")[四元数于三维旋转]. Krasjet. CC BY-NC-SA 4.0
