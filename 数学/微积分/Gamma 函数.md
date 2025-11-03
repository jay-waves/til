Gamma 函数, 即 $\Gamma(n)$, 定义为: $$\Gamma(n)=\int_{0}^{\infty}x^{n-1}e^{-x}dx$$

令 $x=t^{2}$, 则有变形: $$\Gamma(n)=2\int_{0}^{\infty}t^{2n-1}e^{-t^{2}}dt$$

$\Gamma$ 函数定义在复数域上, 除了负整数和零点. 满足递推关系: $$\Gamma(n+1)=n\cdot\Gamma(n)$$

当 $n$ 为整数时, $\Gamma(n+1)=n!$. 因此, $\Gamma$ 函数可以视作阶乘的连续扩展.

$\Gamma$ 函数的特殊值:
- $\Gamma(1)=1$
- $\Gamma\left( \frac{1}{2} \right)=\sqrt{\pi}$

### Euler积分

$\Gamma$ 函数也可通过 Euler 积分表示:

$$\Gamma(z)=\int_{0}^{1}(-\ln t)^{z-1}dt$$
