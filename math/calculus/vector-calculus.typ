#import "../../appx/theme.typ": tufte, note

#show: tufte

#let bmat(..args) = $mat(delim: "[", ..args)$
#let vmat(..args) = $mat(delim: "|", ..args)$
#let vecb(x) = $upright(bold(#x))$

= 标量场

== 梯度（Gradient） 

*标量场* $f(x,y,z)$ 的梯度为：

$
nabla f = vec(display(frac(partial f, partial x)),
display(frac(partial f, partial y)),
display(frac(partial f, partial z)))
$

*梯度总指向函数增长最快的方向，模长代表该点最大的变化化率*。

梯度总垂直于等值面 $f(x,y,z)=c$。如果曲面由：

$ f(x, y, z)=c $

定义，则其法向量可以取为：

$ vecb(n) = nabla f $

== 全微分（一阶泰勒近似） 

在点 $vecb(a)=(a,b)$ 处的线性近似：

$ 
Delta f
&= f(x,y) - f(a,b) \
&approx frac(partial f, partial x) (a,b)(x-a) + frac(partial f, partial y) (a,b) (y-b) \
&= nabla f(vecb(a)) dot (vecb(x-a)) 
$

记 $f(x,y)$ 的*全微分*为：

$ d f = frac(partial f, partial x)d x + frac(partial f, partial y)d y $

因此，全微分是函数准确增量 $Delta f$ 的一阶近似： 

$ Delta f approx d f $

== 方向导数

函数沿单位向量 $upright(bold(u))$ 的*方向导数*：

$ D_(vecb(u))f = nabla f dot vecb(u) $

它表示从某一点沿指定方向移动时，函数变化得有多快。

由于

$ nabla f dot vecb(u) = | nabla f | cos theta $

所以当 $vecb(u)$ 与梯度方向相同时，方向导数最大。

== Hessian Matrix

标量函数 $f: RR^n -> RR^n$ 的二阶偏导数组成 Hessian：

$
H_(f)= bmat(
  frac(partial^(2) f, partial x_(1)^(2)), frac(partial^(2) f, partial x_(1) partial x_(2)) ;
  frac(partial^(2) f, partial x_(2) partial x_(1)), frac(partial^(2) f, partial x_(2)^(2)))
$


梯度用于寻找下降方向；而 Hessian 用于判断极值类型。
若 Hessian 正定，函数在该点附近通常是局部极小；负定则通常是局部极大。

== 二阶泰勒近似 

记 $vecb(h)=(x-a,y-b)^top=vecb(x-a)$

$ 
Delta f approx nabla f (a,b)dot vecb(h) + 1/2 vecb(h)^(T)H_(f)(a,b)vecb(h)  + o(norm(h^2))
$

== 拉普拉斯算子

标量场 $f$ 的拉普拉斯为：

$
nabla^2 f = nabla dot (nabla f)
$

在直角坐标系中，有：

$ 
nabla^2 f = frac(partial^(2)f, partial x^(2)) 
  + frac(partial^(2)f, partial y^(2)) 
  + frac(partial^(2)f, partial z^(2))
$

它衡量某一点的函数值相对于周围值的偏离程度。



= 向量场

== 向量函数

空间曲线通常写成：

$ vecb(r)(t)= vec(x(t), y(t), z(t)) $

单位切向量：

$ vecb(tau) (t)= frac(vecb(r) '(t), | vecb(r) '(t)|) $

== Jacobian Matrix 

对于向量函数：

$ vecb(F) : RR^(n)-> RR^(m) $


雅各比矩阵为：

$
J_(vecb(F)) = bmat(
  frac(partial F_(1), partial x_(1)), dots.h.c, frac(partial F_(1), partial x_(n));
  dots.v,, dots.v;
  frac(partial F_(m), partial x_(1)), dots.h.c, frac(partial F_(m), partial x_(n)))
$

它表示非线性映射在某一点附近的局部线性近似：

$
vecb(F) (vecb(x) + Delta vecb(x) )
approx vecb(F) (vecb(x)) + J_vecb(F)(vecb(x)) Delta vecb(x)
$

因此，雅各比矩阵描述局部的拉伸、压缩、旋转和剪切。

== 散度（Divergence）

向量场

$ vecb(F) = vec(F_(x), F_(y), F_(z)) $

的散度为：

$ 
nabla dot vecb(F) = 
  frac(partial F_(x), partial x) + frac(partial F_(y), partial y) + frac(partial F_(z), partial z) 
$

结果是标量。

散度表示向量场在某点附近的净流出程度：

- $nabla dot vecb(F)>0$ ：局部像源；
- $nabla dot vecb(F)<0$ ：局部像汇；
- $nabla dot vecb(F)=0$ ：局部没有净流出。

在流体力学中，散度常用于判断流体是否可压缩。

== 旋度(Curl)

三维向量场的旋度（向量）为：

$
nabla times vecb(F) = bmat(
  frac(partial F_(z), partial y) - frac(partial F_(y), partial z);; 
  frac(partial F_(x), partial z) - frac(partial F_(z), partial x);;
  frac(partial F_(y), partial x) - frac(partial F_(x), partial y))
$

它表示向量场的局部旋转趋势：

* 方向表示旋转轴；
* 大小表示旋转强度。

#linebreak()

梯度场的旋度为零：

$ nabla times (nabla f) = 0 $

旋度场的散度为零：

$ nabla dot (nabla times vecb(F)) = 0 $


