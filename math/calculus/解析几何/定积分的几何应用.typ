#import "../../../appx/theme.typ": tufte, note

#show: tufte

#let bmat(..args) = $mat(delim: "[", ..args)$
#let vmat(..args) = $mat(delim: "|", ..args)$

== 旋转体体积
#link("https://www.zhihu.com/question/378070928") ???

核心思路是, 将体积微元当成空心圆锥,用高乘以圆环面积

这一思想可以推广至极坐标下的任何旋转体积, 这样求可以利用极坐标的优点规避掉传统用微圆柱求体积时很难处理的空心问题.

== 曲线弧长微元
要求曲线光滑, 即 $d x^2 + d y^2 eq.not 0$. 函数光滑的严格定义是无限阶可导.

== 曲率
$ K = | frac(d alpha, d s) | $

其中角度微元为 $d alpha = frac(d s, r)$, 因而 $K = frac(| d alpha |, d s) = 1 / r$. 曲率和半径负相关, 半径越大, 曲率 (弯曲程度) 越小. 由此将 $rho = 1 / K$ 称为该处曲率圆的半径.

在 `x0y` 坐标系下, 有:

$ K = frac(| y'' |, ( 1 + y' ""^2 )^(3 / 2)) $




