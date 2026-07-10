#import "../../../appx/theme.typ": tufte, note

#show: tufte

#let bmat(..args) = $mat(delim: "[", ..args)$
#let vmat(..args) = $mat(delim: "|", ..args)$

== 正态分布
正态分布, 也称为高斯分布, 记为 $X tilde.op N(mu , sigma^2)$, 其概率密度函数为:

$ f(x) = frac(1, sigma sqrt(2 pi)) e^(- frac(( x - mu )^2, 2 sigma^2)) $

$E X = mu$, $D X = sigma^2$.

=== 概率密度函数
以 $mu$ 为对称轴,

=== 可加性
若正态分布 $X tilde.op N(mu_1 , sigma_1^2) , Y tilde.op N(mu_2 , sigma_2^2)$ 相互独立, 则 $ X + Y tilde.op N(mu_1 + mu_2 , med sigma_1^2 + sigma_2^2) $

== 标准正态分布
称满足 $mu = 0 , sigma = 1$ 为标准正态分布 $X tilde.op N(0 , 1)$, 此时概率密度函数为: $ f(x) = 1 / sqrt(2 pi) e^(- x^2 / 2) $

正态分布到标准正态分布的转换为: $Z = frac(X - mu, sigma)$, 即 $F(z) = Phi(frac(X - mu, sigma)), X tilde.op N(mu, sigma^2)$.

== 二维正态分布
若 $X , Y tilde.op N(mu_1 , mu_2 ; sigma_1^2 , sigma_2^2 ; rho)$ 不独立, 则两者的二维正态分布概率密度函数为:

$ f(x , y) = dots.h $





