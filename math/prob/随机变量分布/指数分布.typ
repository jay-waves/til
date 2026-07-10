#import "../../../appx/theme.typ": tufte, note

#show: tufte

#let bmat(..args) = $mat(delim: "[", ..args)$
#let vmat(..args) = $mat(delim: "|", ..args)$

== 指数分布
在随机质点流计数过程中 (视为泊松过程, $N_t tilde.op P(lambda t)$), 第一个质点到来时刻为 $t$ 的概率为 $P_t = e^(- lambda t)$, 第一个质点在时间 $t$ 之前到来的概率为 $P = 1 - P_t = 1 - e^(- lambda t)$. 将时间视为连续的, 则有指数分布 $X tilde.op E(lambda)$.

指数分布也称为零件失效等待频率, $ lambda$ 是失效频率. 在泊松过程中参数 $lambda_P$ 是较长时间内的发生次数, 指数分布中的参数 $ lambda_E = lambda_P div t $ 是发生概率.

概率密度分布函数: $f(x) = {lambda e^(- lambda x) , med x > 0\
0$

概率分布函数为: $F(x) = {1 - e^(- lambda x) , med x gt.eq 0\
0$

期望为 $E X = 1 / lambda$, 方差为 $D X = 1 / lambda^2$, 无记忆性为 $P = P lr({X gt.eq t + s bar.v X gt.eq t}) = P lr({X gt.eq s})$





