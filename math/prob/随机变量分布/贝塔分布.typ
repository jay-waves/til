#import "../../../appx/theme.typ": tufte, note

#show: tufte

#let bmat(..args) = $mat(delim: "[", ..args)$
#let vmat(..args) = $mat(delim: "|", ..args)$

== F(F(X))
给定随机变量 $X$ 及其连续的严格单调增加的累积分布函数 $F(x)$, 讨论 $F(X)$ 的分布.

由于 $F(x)$ 是连续的, 也是严格递增的, 所以在定义域上可逆. 令 $U = F(X) in [ 0 , 1 ]$.

$ F_U(u) & = P(U lt.eq u)\
 & = P ( F(X) lt.eq u )\
 & = P ( X lt.eq F^(- 1)(u) )\
 & = F ( F^(- 1)(u) )\
 & = u $

因此 $U$ 在 $[ 0 , 1 ]$ 上服从均匀分布.




