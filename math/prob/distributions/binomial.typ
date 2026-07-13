#import "../../../appx/theme.typ": tufte, note

#show: tufte

#let bmat(..args) = $mat(delim: "[", ..args)$
#let vmat(..args) = $mat(delim: "|", ..args)$

== 伯努利分布
#strong[伯努利分布, 也叫 0-1 分布]. 其中随机变量 $X$ 表示单次随机试验中只有两个可能结果: - 成功, 1, $P(X = 1) = p$ - 失败, 0, $P(X = 0) = 1 - p$

$E X = p$, $D X = p(1 - p)$

== 二项分布
二项分布, ($upright("Ber") - E_n$, n 重伯努利试验). 随机变量 $X$ 表示 n 次独立的伯努利试验中成功的次数, 则 $X$ 服从参数为 $n , p$ 的二项分布, 记为 $X tilde.op B(n , p)$.

$ P(X = k) = binom(n, k) p^k(1 - p)^(n - k) $

$E X = n p$, $D X = n p(1 - p)$

若两个服从二项分布的随机变量 $X , Y$ 相互独立, 两者可加. 单次实验可以看作单位时间 $Delta t$ 内的事件概率.

== 多项式分布




