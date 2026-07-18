#import "../../appx/theme.typ": tufte, note

#show: tufte

#let bmat(..args) = $mat(delim: "[", ..args)$
#let vmat(..args) = $mat(delim: "|", ..args)$

= 微分

导数是函数在某个方向的变化率, 定义为:

$ f'(x) = lim_(Delta x arrow.r 0) frac(f(x + Delta x) - f(x), Delta x) $

某点导数存在等价于:

$ f'_(+)(x) = f'_(-)(x) = f(x) $

可微是映射的局部线性近似

$ f(x) - f(x_0) = f'(x_0) ( x - x_0 ) + o(x - x_0) $

$ d y = f'(x_0) ( x - x_0 ) = A(x - x_0) $

英文教材中常说 ” 可微 ” 而不是 ” 可导 ", 中文习惯说" 可导 “, 单变量函数中可导=可微.

== 常见导数表
#table(
    columns: 2,
    align: (auto,auto,),
    table.header([函数], [导数],),
    table.hline(),
    [$C$], [$0$],
    [$x^mu$], [$mu dot x^(mu - 1)$],
    [$a^x$], [$a^x ln a$],
    [$log_a x$], [$frac(1, x ln a)$],
    [$sin x$], [$cos x$],
    [$cos x$], [$- sin x$],
    [$tan x$], [$sec^2 x = 1 + t a n^2 x$],
    [$sec x$], [$sec x dot tan x$],
    [$csc x$], [$- csc x dot cot x$],
    [$arcsin x$], [$1 / sqrt(1 - x^2)$],
    [$arccos x$], [$- 1 / sqrt(1 - x^2)$],
    [$arctan x$], [$frac(1, 1 + x^2)$],
  )

= 求导规则
== 函数
设 u, v 皆对 x 可导:

$ ( C dot u )' = C u' $

$ ( u + v )' = u' + v' $

$ ( u v )' = u' v + u v' $

$ (u / v)' = frac(u' v - u v', v^2) $

== 反函数定理
设 $x = phi(y)$ 在区间 $I_y$ 可导, 单调, 且 $phi'(y) eq.not 0$. 且反函数 $y = f(x)$ 在对应 $I_x$ 也可导:

$ f'(x) = frac(1, phi'(y)) $ 或 $ frac(d y, d x) = 1 / frac(d x, d y) $

= 可微与可导的关系

$ f(x) = x / x = cases(1 & x eq.not 0, "未定义" & x = 0) $

在严格分数定义中, 分母为零未定义. 只有有定义的时候, 才能通过上下约分来化简. 实际应用中, 用 $lim_(x arrow.r 0) x / x = 1$ 来延拓定义 $x = 0$ 处的行为.

$ x dot sin(1 / x) = cases(1 & x arrow.r 0, "未定义" & x = 0) $

这些函数在 $x_0$ 上没有定义, 但是趋近于 $x_0$ 的去心邻域上极限是存在的. 这导致了现代极限定义#strong[只在去心邻域上考虑极限问题, 而不考虑 $x_0$ 点本身的取值 (甚至是否有定义)]: $lim_(x arrow.r x_0) f(x) = A$ 当且仅当 $forall epsilon > 0 , med exists delta > 0 , quad s . t . quad 0 < | x - a | < delta arrow.r.double | f(x) - A | < epsilon$

#line(length: 100%)

== 一元函数

#note[图示见 `./differentials-and-derivatices.md`]

左右极限存在且相等, 说明函数在该点#strong[极限存在]: $ lim_(x arrow.r a^(+)) f(x) = lim_(x arrow.r a^(-)) f(x) $

左右极限存在且等于该点定义值, 说明函数在该点局部#strong[连续]: $ lim_(x arrow.r a^(+)) f(x) = lim_(x arrow.r a^(-)) f(x) = f(a) $

左右导数存在且相等, 说明函数在该点局部#strong[可导]: $ lim_(x arrow.r a^(+)) f'(x) = lim_(x arrow.r a^(-)) f'(x) $

== 多元函数

#note[图示见 `./differentials-and-derivatices.md`]

对于多元函数, 偏导存在是指 $x , y$ 正方向上导数存在, 由于不关心其他方向, 因此偏导可能不连续.

#strong[若二阶偏导 $frac(partial^2 z, partial x partial y)$, $frac(partial^2 z, partial y partial x)$ 都在区域 D 内连续, 则] $ frac(partial^2 z, partial x partial y) = frac(partial^2 z, partial y partial x) $

#strong[若 $f(x , y)$ 光滑 (偏导连续), 则] $ frac(partial, partial x) integral_v^u f(x , y) thin d y = integral_v^u frac(partial f(x , y), partial x) thin d y $

= 隐函数存在定理
$F(x , y) = 0$ 可确定隐函数 $y = f(x)$, 且 $ F'_y(x , y) eq.not 0 $, 有 $0 = frac(partial F, partial x) dif x + frac(partial F, partial y) dif y$ $frac(d y, d x) = f'(x) = - frac(F'_x, F'_y)$

$F(x , y , z) = 0$ 可确定隐函数 $z = f(x , y)$, 且 $F'_z(x , y , z) eq.not 0$, 有

$ cases(F'_x + F'_z(x, y, z) dot frac(partial z, partial x) = 0, F'_y + F'_z(x, y, z) dot frac(partial z, partial y) = 0) $

$ cases(frac(partial z, partial x) = - frac(F'_x, F'_z), frac(partial z, partial y) = - frac(F'_y, F'_z)) $




