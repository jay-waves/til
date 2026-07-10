#import "../../appx/theme.typ": tufte, note

#show: tufte

#let bmat(..args) = $mat(delim: "[", ..args)$
#let vmat(..args) = $mat(delim: "|", ..args)$

=== 贝叶斯定理
$ P(A | B) = frac(P(B | A) times P(A), P(B)) $

- P(A) 是#strong[先验概率 (Prior Probability)], 表示在获得新证据 (B) 之前对某事件发生概率的初始估计.
- P(A|B) 是#strong[后验概率 (Posterior Probability)], 表示获取新证据 B 之后, 对初始估计进行修正后的概率.
- P(B|A) 是在已知A的条件下B发生的概率, 也被称为似然度 (Likelihood).
- P(B) 是B发生的总概率, 也叫作边缘概率.

贝叶斯定义演化出#strong[贝叶斯决策(推理)理论 (Bayesian Decision Theory)]: 对于 $w_1 , w_2 , dots.h , w_c$ 种类别的分类问题, $w_i$ 出现的先验概率为 $P(w_i)$. 如果在特征空间观察到(证据)向量 $overline(x) = [ x_1 , x_2 , dots.h , x_d ]^T$, 且 $P(x | w_i)$ 是已知的, 那么可以得到 $w_i$ 出现的后验概率为: $ P ( w_i | overline(x) ) = frac(P ( overline(x) | w_i ) P(w_i), sum_(j = 1)^n P ( overline(x) | w_j ) P(w_j)) $ 基于最小错误率的贝叶斯决策规则为: #strong[如果 $P ( w_i | overline(x) ) = max_(j = 1 , 2 , dots.h , c) P ( w_j | overline(x) )$, 那么 $overline(x) in w_i$]

=== 贝叶斯公式的全概率展开
$ P(B | A) = frac(P(A dot B), P(A)) = frac(P(A | B) P(B), P(A)) $

其中 P(A) 即观测事件 A 的概率, 可以通过#strong[全概率公式:] $P(A) = sum_i P(A | B_i) P(B_i)$ 展开. 其中 $B_i$ 是一组互斥且完备的事件 (即 $B_i$ 的并集是整个样本空间, 且任意两事件不可能同时发生).

得到:

$ P(B_j | A) = frac(P(A | B_j) P(B_j), sum_i P(A | B_i) P(B_i)) $




