#import "../../appx/theme.typ": tufte, note

#show: tufte

#let bmat(..args) = $mat(delim: "[", ..args)$
#let vmat(..args) = $mat(delim: "|", ..args)$

== 切比雪夫不等式
设随机变量 $X$ 具有期望 $E X = mu , D X = sigma^2$, 对于任意正数 $epsilon$, 满足不等式:

$ P { | X - mu | gt.eq epsilon } lt.eq sigma^2 / epsilon^2 $

等价于 $ P { | X - mu | < epsilon } gt.eq 1 - sigma^2 / epsilon^2 $

#strong[证明]:

$ P { | X - mu | gt.eq epsilon } & = integral_(| X - mu | gt.eq epsilon) f(x) thin d x\
 & lt.eq integral_(| X - mu | gt.eq epsilon) frac(| X - mu |^2, epsilon^2) f(x) thin d x\
 & lt.eq 1 / epsilon integral_(- oo)^oo(x - mu)^2 f(x) thin d x\
 & = sigma^2 / epsilon^2 $

$square.filled.medium$

取 $epsilon = 2 sigma$, 于是: $ P { | X - mu | < 2 sigma } gt.eq 1 - frac(sigma^2, ( 2 sigma )^2) = 3 / 4 $

类似地可以估计 $X$ 在 $n sigma^2$ 范围内的概率值.





