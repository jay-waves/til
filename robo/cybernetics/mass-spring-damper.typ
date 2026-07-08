#import "../../template.typ": tufte, note
#show: tufte


= 线性 质量-弹簧-阻尼 模型

#figure(
  image("../../attach/mass-spring-damper.webp", width: 60%),
  caption: [Modern RObotics fig 11.5],
)

设参数如下：
- 质量 $m$ 
- 位置 $theta$ 
- 阻尼施加的力 $-b dot(theta)$ ，$b$ 是阻尼常数
- 弹簧施加的力 $-k(theta)$ ，$k$ 是弹簧刚度
- 外力 $f$ 

力学公式为： $ m dot.double(theta)+b dot(theta)+k theta=f $

= 无外力情况

在无外力情况下 ($f=0$) ，考虑如何控制误差稳定，并收敛为零。

#math.equation(
  numbering: "(1)",
  supplement: "Eq.",
  block: true,
  $ dot.double(theta)(t) + frac(b, m) dot(theta)(t)+frac(k,m)theta (t)=0 $ 
) <eq1> 

设自然频率 (natrual frequency) $w_(n)=sqrt(k/m)$ ，阻尼系数 (damping ratio) $zeta=b/(2sqrt(k m))$， @eq1 式的特征多项式为：

$ s^(2)+2 zeta omega_(n)s+omega_n^2=0 $

得到两个根：

$ 
s_(1)& = - zeta omega_(n)+ omega_(n)sqrt(zeta^(2)- 1) \ 
s_(2) & = - zeta omega_(n)- omega_(n)sqrt(zeta^(2)- 1) 
$


要让 @eq1 式稳定，即 $theta_(t->infinity )=0$ ，等价于让所有特征值满足 $Re(s_(i))<0$ 等价于 $zeta omega_n >0$。

当 $theta$ 稳定时，有三种解：
+ $zeta > 1$ 时，$s_1,s_2$ 是实数且不相等。称为*过阻尼 (Overdamped)*，系统慢收敛。
+ $zeta =1$ 时，$s_1,s_2$ 是实数且相等。称为*临界阻尼*，系统最快收敛。
+ $zeta < 1$ 时，$s_1,s_2$ 是复数。称为*欠阻尼 (Underdamped)*，系统遮挡并指数衰减。



== 阻尼控制 

11.7 看完

= 参考

Modern Robotics -- Kevin Lynch & Frank C. Park -- 2019 

#link("https://en.wikipedia.org/wiki/Mass-spring-damper_model")[Mass-spring-damper model -- wikipedia]

