#import "../../appx/theme.typ": tufte, note
#show: tufte

= 符号的定义

- $omega$ 模拟信号（连续时间的）角频率，单位为 *弧度每秒* $r a d \/ s$
- $Omega$ 数字信号（离散时间的）归一化角频率
- $f$ 物理频率，满足 $omega=2pi f$
- $f_s$ 采样频率，见#link("./signals.md")[信号采样一节]
- $t$ 线性系统的时间

= 常见序列

*单位抽样序列*：

$
  delta(t) = cases(
   1 "if" t = 0,
   0 "if" eq.not 0 
  )
$

$
  x(m) delta (t-m) = cases(
    x(t) "if" t = m,
    0 "if" dots
  )
$

*单位阶跃序列：* 

$ u(t) = cases(
  1 "if" t >= 0,
  0 "if" t < 0
) $


*矩形序列：* 

$ R_N (t) = cases( 1 "if" N-1 >= t >= 0, 0 "if" t < 0) $ 

= 卷积

$
y(t)=x(t) * h(t)=sum^(+ infinity)_(m = -infinity) x(m) dot h(t-m)=sum x(t-m)dot h(m)
$

$h(t)$ 表示某 [LSI 系统](signals.md), *卷积* 代表 $x(t)$ 经过 $h(t)$ 得到的结果.  

= 相关

$ r_(x y)(n)=sum^(infinity)_(m=-infinity) x(m)dot y(m-n)=x(n)*y(-n) $

*相关*指两个信号之间的相互关系.

卷积满足交换律, 互相关函数不满足交换律.


= 卷积定理

*时域卷积等于频域乘积，时域乘积等于频域卷积*

$ cal(F){f*g}=F(omega)G(omega) $

$ cal(F){f(t)g(t)}=1/(2pi)(F*G)(omega) $