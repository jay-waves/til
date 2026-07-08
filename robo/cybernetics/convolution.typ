#import "../../template.typ": tufte, note
#show: tufte

= 卷积

$
y(t)=x(t)*h(t)=sum^(+ infinity)_(m = -infinity) x(m) dot h(t-m)=sum x(t-m)dot h(m)
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