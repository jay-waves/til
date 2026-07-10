#import "../../appx/theme.typ": tufte, note
#show: tufte

雅各比坐标系 (Jacobian coordinates) 转换

#let pd(f, x) = $frac(partial #f, partial #x)$
#let dx(x) = $dif #x$
#let pd(i, j) = $frac(partial #i, partial #j)$

= 雅各比矩阵

定义可微函数: 

$ bold(F)(bold(x)) = (F_1(bold(x)), F_2(bold(x)), ..., F_n(bold(x))) $

其中 $bold(x) = (x_1, x_2, ..., x_n)$。

雅各比矩阵 $J_(n times n)$ 定义为：

$
J = mat(
  pd(F_1, F_1), pd(F_1, F_2), dots.h, pd(F_1, F_n);
  pd(F_2, F_1), pd(F_2, F_2), dots.h, pd(F_2, F_n);
  dots.v, dots.v, dots.down, dots.v;
  pd(F_n, F_1), pd(F_n, F_2), dots.h, pd(F_n, F_n);
)
$

= 坐标变换

多变量积分中, 雅各比行列式可以用于换元. 考虑 $bold(x)=G(bold(u))$, 积分微元使用雅各比行列式进行
调整: $ d bold(x)=|J|d bold(u) $

所以积分换元表示为: 

$ integral_A f(bold(x)) = integral_B f(G(bold(u))) abs(J) dx(bold(u)) $

, 其中 $d$ 指外微分算子, $dx(x)=dx(x_1) and dx(x_2) and dots and dx(x_n)$, 在多变量积分中可理解为微元乘积 $dx(x_1)dx(x_2)dots dx(x_n)$, 也是微元体积.

*证明*:

...