#import "@local/ypst-template:0.1.0": template, sidenote

#show: template

= 采样

在数字信号处理中，将连续时间信号 $x(t)$ 以采样频率 $f_s$ 采样，产生离散时间信号：

$
x[n] = x(n T_s) = x(n frac(1, f_s))
$

== 模拟角频率与数字角频率

模拟信号角频率 $omega$ 和数字信号角频率 $Omega$ 之间的关系是：

$
Omega = omega T_s = frac(omega, f_s)
$

由于：

$
omega = 2 pi f, quad omega_s = 2 pi f_s
$

因此：

$
Omega = 2 pi frac(f, f_s), quad Omega = 2 pi frac(omega, omega_s)
$

数字角频率 $Omega$ 以弧度每采样点（rad/sample）为单位。

在离散时间傅里叶变换（DTFT）中，频谱以 $2 pi$ 为周期，通常取 $[-pi, pi]$ 作为数字角频率的主值区间。


== 频谱信号

带限信号 $x(t)$，通过傅里叶变换得到频谱 $X(f)$。

由于是带限的，其频谱只存在于有限频率范围：

$
X(f) = 0, quad "if" abs(f) > f_"max"
$


== 采样信号

将连续时间信号 $x(t)$ 以间隔 $T_s$ 进行采样。

利用连续时间单位冲激函数 $delta(t)$，采样后的冲激信号表示为：

$
x_s (t)
  &= x(t) dot sum_(n=-infinity)^infinity delta(t-n T_s) \
  &= sum_(n=-infinity)^infinity x(n T_s) delta(t-n T_s)
$

注意：这里的 $delta(t)$ 是连续时间 Dirac 冲激函数，不同于前面的离散时间单位抽样序列 $delta(n)$。

将采样信号进行傅里叶变换：

$
X_s (f) = cal(F)(x_s (t))
$

采样后的信号在频域是原始信号频谱的周期性重复，频谱重复周期为：

$
f_s = frac(1, T_s)
$

因此：

$
X_s (f)
  = frac(1, T_s) sum_(k=-infinity)^infinity X(f-k f_s)
$


= 奈奎斯特采样定理

奈奎斯特采样定理（Nyquist Sampling Thm.）指出，要完整重建一个带限信号，采样频率需要满足：

$
f_s > 2 f_"max"
$

如果采样频率低于该标准，信号中高频分量就可能被误解释为低频分量，导致*混叠*（Aliasing）。

由于 $X_s (f)$ 是 $X(f)$ 在频谱上以 $f_s$ 为周期的重复，为保证周期性重复的频谱不发生重叠，需要满足上述条件。


== 信号重建

用理想低通滤波器从采样信号中提取原始信号 $x(t)$。#footnote[低通滤波器详见 ./low-pass-filter.md]

对于满足采样定理的带限信号，可通过 sinc 插值重建：

$
x(t)
  = sum_(n=-infinity)^infinity
    x(n T_s)
    dot op("sinc")(
      frac(t-n T_s, T_s)
    )
$

其中，采用归一化 sinc 函数：

$
op("sinc")(x) = frac(sin(pi x), pi x)
$

$sinc$ 函数用于插值重建原始信号。

= 离散化

采样将连续信号 $x(t)$ 变为离散采样点 $x[n]=x(n T_s)$ ，然后，需要将
连续时间的动态关系转换为差分方差。

== 移不变系统的精确离散化

系统（其中，y 为输出，x 为状态，u 为输入）：$ frac("d"x(t), "d"t) = A x(t) + B u(t) $

输出 $ y(t) = C x(t) + D u(t) $

采样 $ x[k] = x(k T_s) $

*假设输入 $u$ 在一个周期内保持不变*：$ u(t) = u[k] $ 

求解连续系统微分方程 $ dot(x) = A x + B u$，得到：

$ x[k+1] = e^(A T_s) med x[k] + integral^(T_s)_0 e^(A tau) B upright(d) tau med u[k] $

=== 移不变系统给近似离散化

使用前向欧拉法#footnote[详见数值计算相关内容 `../../math/numerical/ode-initial-value-problems.typ`]逼近离散化后的线性系统的近似解：

设离散系统：$ dot(x)(t) = f(x(t), u(t)) $

用差分近似：$ dot(x)(t) approx frac(x[k+1] - x[k], T_s) $ 

联立得到： $ x[k+1] = x[k] + T_s f(x[k], u[k]) $

对于线性系统 $dot(x) = A x + B u$ ，有：$ x[k+1] = (I+A T_s) x[k] + T_s B u[k] $
