#import "../../appx/theme.typ": tufte, note

#show: tufte

#let bmat(..args) = $mat(delim: "[", ..args)$
#let vmat(..args) = $mat(delim: "|", ..args)$

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

== 2 求导规则
=== 函数
设 u, v 皆对 x 可导:

$ ( C dot u )' = C u' $

$ ( u + v )' = u' + v' $

$ ( u v )' = u' v + u v' $

$ (u / v)' = frac(u' v - u v', v^2) $

=== 反函数定理
设 $x = phi(y)$ 在区间 $I_y$ 可导, 单调, 且 $phi'(y) eq.not 0$. 且反函数 $y = f(x)$ 在对应 $I_x$ 也可导:

$ f'(x) = frac(1, phi'(y)) $ 或 $ frac(d y, d x) = 1 / frac(d x, d y) $

==== 二阶反函数定理
…

== 3 常见导数推导





