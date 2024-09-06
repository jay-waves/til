IPA: 证明知晓满足 $\vec{c}=\vec{a}\cdot\vec{b}$ 的一个解: $\vec{a}$ 和 $\vec{b}$, 而不透具体解值.

IPA 具体有如下步骤:
- 承诺阶段: Prover 将证明值封装入承诺, 发送给 Verifier
- 挑战阶段: Verifier 发送一个随机挑战给 Prover
- 响应阶段: Prover 发送响应 (通常是完成一个 inner product prove) 给 Verifier
- 验证: Verifier 使用先前承诺, 来验证该响应.

最初的承诺相当于 prover 给 verifier 的一个人质, 防止作弊.

传统 IPA 由 Pedersen 承诺实现, 见论文 [BulletProofs](../文档/BulletProofs.pdf).

***

在 ZKRP 中, IPA 要证明值在 $[0, 2^{n}-1]$ 范围内.

将值转化为二进制向量, 即证:  
1. 向量 $v$ 所有元素都是01: $v \odot (v-1^{m})=0^{m}$
2. 向量 $v$ 前 $m-n$ 位皆为0: $v \odot (1^{m-n}\Vert 0^{n})=0^{m}$

对于**随机**向量 $r\in \mathbb{F}^{m}$, 如果 $<a, r>=0$, 那么 $a\neq 0^{m}$ 的概率小于 $\frac{1}{\vert\mathbb{F}\vert}$. 由此, 上式转化为:
1. $\langle v\odot (v-1^{m}),\ r\rangle=0$
2. $\langle v\odot(1^{m-n}\Vert 0^{n}),\ r\rangle=\langle v,\ r_{[:m-n]}\Vert 0^{n}\rangle=0$

[BulletProofs](BulletProofs.md) 中, 使用的 Pedersen 承诺方法仅满足半哈达玛积同态 (公开向量积隐私向量), 可以把 $\langle v\odot (v-1^{m}),\ r\rangle=0$ 转化为 $\langle v, (v-1^{m}) \odot r\rangle=0$, 用 $v$ 和 $(v-1^{m})\odot r$ 构造承诺.

***

> 待阅读: 
> [阅读wiki1](https://suyash67.github.io/homepage/project/2020/06/28/inner-product-argument.html), 
> [Proof for IPA](../文档/Proof%20for%20IPA.pdf)

