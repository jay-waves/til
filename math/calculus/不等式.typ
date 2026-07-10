#import "../../appx/theme.typ": tufte, note

#show: tufte

#let bmat(..args) = $mat(delim: "[", ..args)$
#let vmat(..args) = $mat(delim: "|", ..args)$

== 均值不等式
#strong[平均數不等式], 或称#strong[平均值不等式], #strong[均值不等式]. 是算术-几何平均不等式的推广.

$ x_1 , x_2 , dots.h , x_n in bb(R_(+)) , med frac(n, sum_(i = 1)^n 1 / x_i) lt.eq root(n, product_(i = 1)^n x_i) lt.eq frac(sum_(i = 1)^n x_i, n) lt.eq sqrt(frac(sum_(i = 1)^n x_i^2, n)) $

即: $ H_n lt.eq G_n lt.eq A_n lt.eq Q_n $

其中:

$ H_n = frac(n, sum_(i = 1)^n 1 / x_i) = frac(n, 1 / x_1 + 1 / x_2 + dots.c + 1 / x_n) $

$ G_n = root(n, product_(i = 1)^n x_i) = root(n, x_1 x_2 dots.c x_n) $

$ A_n = frac(sum_(i = 1)^n x_i, n) = frac(x_1 + x_2 + dots.c + x_n, n) $

$ Q_n = sqrt(frac(sum_(i = 1)^n x_i^2, n)) = sqrt(frac(x_1^2 + x_2^2 + dots.c + x_n^2, n)) $

当前仅当: $x_1 = x_2 = x_3 = dots.h = x_n$, 等号成立.

可以理解为: 调和平均数 $lt.eq$ 几何平均数 $lt.eq$ 算术平均数 $lt.eq$ 平方平均数.




