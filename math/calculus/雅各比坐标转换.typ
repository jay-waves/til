#import "../../appx/theme.typ": tufte, note

#show: tufte

#let bmat(..args) = $mat(delim: "[", ..args)$
#let vmat(..args) = $mat(delim: "|", ..args)$

雅各比坐标系 (Jacobian coordinates) 转换

=== 雅各比矩阵
定义可微函数: $F ( upright(bold(x)) ) : bb(R)^n arrow.r bb(R)^n$ 为 $upright(bold(F)) ( upright(bold(x)) ) = ( F_1 ( upright(bold(x)) ) , F_2 ( upright(bold(x)) ) , dots.h , F_n ( upright(bold(x)) ) )$, 其中 $upright(bold(x)) = ( x_1 , x_2 , dots.h , x_n )$.

雅各比矩阵 $J_(n times n)$ 定义为: $ J = mat(frac(partial F_1, partial x_1), frac(partial F_1, partial x_2), dots.c, frac(partial F_1, partial x_n); frac(partial F_2, partial x_1), frac(partial F_2, partial x_2), dots.c, frac(partial F_2, partial x_n); dots.v, dots.v, dots.down, dots.v; frac(partial F_n, partial x_1), frac(partial F_n, partial x_2), dots.c, frac(partial F_n, partial x_n)) $

=== 坐标变换
多变量积分中, 雅各比行列式可以用于换元. 考虑 $upright(bold(x)) = G ( upright(bold(u)) )$, 积分微元使用雅各比行列式进行 调整: $ d upright(bold(x)) = | J | d upright(bold(u)) $

所以积分换元表示为: $ integral_(upright(bold(A))) f ( upright(bold(x)) ) thin d upright(bold(x)) = integral_(upright(bold(B))) f ( upright(bold(G)) ( upright(bold(u)) ) ) | J | thin d upright(bold(u)) $, 其中 $d$ 指外微分算子, $d upright(bold(x)) = d x_1 and d x_2 and dots.h and d x_n$, 在多变量积分中可理解为微元乘积 $ d x_1 d x_2 dots.h d x_n $, 也是微元体积.

#strong[证明]:

…





