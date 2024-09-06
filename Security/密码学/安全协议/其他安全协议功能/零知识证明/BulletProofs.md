BulletProofs 基于**广义 Pedersen 承诺**来完成 IPA 非交互式证明. 

> [论文 BulletProofs](../文档/BulletProofs.pdf)
> 
> 递归式 IPA 实现 [论文 Halo](../文档/Halo.pdf). 
> 
> 讲解: [Understanding Inner Product Argument – Suyash Bagad](https://suyash67.github.io/homepage/project/2020/06/28/inner-product-argument.html)

### 证明向量由 01 构成

假设 Pedersen 承诺对于 $v$ 为 $C(v)$. 基于此的IPA步骤如下:

1. Prover 发送 $C(v;\ r_{1})$
2. Verifier 发送随机挑战 $\delta$
3. Prover 发送 $C((v-1^{m})\odot \delta;\ r_{2})$

... ==还没看懂==

### Pedersen 承诺

#### 数值 Pedersen 承诺

对于有限群 $G,\ \vert G\vert =p$. $g, y\in G$, $o(g)=p$, 有: $Com(\vec{v};\ r)=y^{r} \cdot g^{v}$.

- Commit: 公开 $q, g, y$, 承诺 $y^{r}\cdot g^{v}$
- Open: 收到 $v,r$, 验证.

其中 $r$ 是一个扰动值. 通过 [离散对数困难问题](../../../../../Math/数论/欧拉定理.md), 保证了:
- $v$ 隐藏
- 给定 $(v, r)$, 难以找到 $(v_{1}, r_{1})$ 有相同的承诺值.

#### 广义 Pedersen 承诺

-   群 $\mathbb{Z}_p^*$, 有生成元: $g$.
-   有随机群元素 $(g_1,\dots,g_n,y)$.
-   通过随机数 $r$ 承诺向量 $\vec{m} = (m_1,\dots, m_n)$, 得到 $c = Com(\vec{m};\ r) = y^r \prod_{i=1}^n g_i^{m_i}$.

可以看出, 在 $\mathbb{Z}_{p}^{n}$ 上向量幂 $\vec{a}^\vec{b}$ 定义为: $(a_{1}^{b_{1}}, a_{2}^{b_{2}} ,a_{3}^{b_{3}},\dots )$.

[广义 Pedersen 承诺的有效性证明 - StackOverflow](https://crypto.stackexchange.com/questions/55955/using-pedersen-commitment-for-a-vector)

令 $\beta \gets \sum_{i=1}^n \alpha_im_{i}=\vec{\alpha}\cdot\vec{m}$, 那么 $c=y^r\prod_{i=1}^n g_i^{m_i} = y^{r+\beta}$.

#### Pedersen 承诺的同态性质

**加法同态性**: 

$C(\vec{a};r_{1})\times C(\vec{b}; r_{2})=C(\vec{a}+\vec{b};\ r_{1}+r_{2})=C(\vec{c};\ r_{1}+r_{2})$ , 即利用公开值 $r_{1}+r_{2}=r_{3}$ 来证明 $\vec{a}+\vec{b}=\vec{c}$.


**哈达玛积同态 (仅限公开向量和保密向量之间运算)**:


ref{author_last_name:year:First Letter for first three title words}