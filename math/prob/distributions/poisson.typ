#import "../../../appx/theme.typ": tufte, note

#show: tufte

#let bmat(..args) = $mat(delim: "[", ..args)$
#let vmat(..args) = $mat(delim: "|", ..args)$

= 泊松分布
设离散型随机变量 $X$ 表示在固定时间, 空间或区间内发生某事件的次数, 尤其在事件发生较稀疏且独立时. 此时 $X$ 服从参数 $lambda$ 的泊松分布, 记为 $X tilde.op upright("Poisson") ( lambda )$.

$ P(X = k) = frac(lambda^k, k !) e^(- lambda) $

$E X = lambda$, $D X = lambda$

== 泊松分布与二项分布
当试验次数 n 较大, 单次试验事件发生概率 $p$ 较小时, 设 $lambda = n p$ 为参数, 假设 $p arrow.r 0 , n arrow.r oo , k lt.double n$.

$ p &= C_n^k dot p^k(1 - p)^(n - k) \ &= frac(n!, (n - k)! dot k!) dot p^k dot (1 - p)^(n - k) \ &= frac((n p)^k, k!) dot frac(1, n^k) dot frac(n!, (n - k)!) dot (1 - frac(lambda, n))^(n - k) \ &= frac(lambda^k, k!) (1 - frac(lambda, n))^(((n - k) / lambda) dot lambda) \ &= frac(lambda^k, k!) e^(-lambda) $

在实际建模中, 将 $n$ 视为时间的度量, $lambda$ 视为事件的平均发生次数. 当两事件的随机变量独立时, 泊松分布也具有可加性.





