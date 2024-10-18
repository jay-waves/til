---
url:  https: //zhuanlan.zhihu.com/p/346563055
author:  dengtesla
date:  22-02-03
---

## 背景

AKS 是第一个**确定性的多项式复杂度的**素性判定算法,  并且不依赖任何未证明猜想.

虽然复杂度是多项式的,  但仍比概率性算法  (如 [Miller -Rabin 算法](素性检测-米勒罗宾方法.md) )  性能差较多. AKS 学术意义比较大.

## 算法

1. 首先给进一个正整数 $n> 1$ 
2. 如果  $n$  是某个数的整数次幂, 那么直接返回"合数"
3. 找到最小的 $r$ 使得 $ord_r (n)  > log^2n$ 
4. 如果存在 $a \leq r$ 使得 $gcd (a, n)  \neq 1 \ or \ n$ , 那么返回"合数"
5. 如果  $n \leq r$  那么返回"质数"
6. 对于 $a \in [1, \lfloor \sqrt{\phi (r) }\log{n} \rfloor]$ 判断 $(X + a) ^n =  (X^n + a)  \  (mod \ X^r - 1,  n)$  是否恒成立. 如果不是, 那么返回"合数"
7. 返回"质数"

### 引理1

如果 $\forall a, n \  (n \geq 2) ,  \ gcd (a, n)  = 1$, 那么  $n$  为质数当且仅当 $$(X+a) ^n = X^n + a \  (mod \ n)$$

**证明**: 

- 先证充分性: 

当 $n$ 为质数时,  $(X+a) ^n$  的展开式中除了  $X^n$  和 $a^n$ 外, 所有的项的系数都包含至少一个  $n$  (把 $C_p^k$ 展开成阶乘的形式即可发现除了分子中的 $p!$ 外其他地方均不可能产生 $p$) , 因此

$(X+a)^n = X^n + a^n \ \pmod n$ 

又由费马小定理可知:  $a^{n-1} = 1\  (mod \ n)$ . 

综上即可得证. 

- 再证必要性: 

若  $n$  不为质数, 那么必然有一个 $n$ 的质因子 $q$ 满足 $q \ne n$ 并且 $q^k|n,  \ k \geq 1$ 

那么考虑 $ (X+a) ^n$ 展开式中 $X^{q}$ 的系数, 为 $C_n^q * a^{n-q}$ . 那么 $q^k$ 显然不是 $C_n^q$ 的因子, 并且与 $a^{n-q}$ 互质. 因此 $X^{q}$ 的系数不为 $0$ , 该式不成立. 

综上, 引理1证毕. 

### 引理2

记 $LCM (m)$ 为 $lcm (1, 2, 3, ..., m)$ , 那么当  $m \geq 7$  时, 一定有 $LCM (m)  \geq 2^m$.

### 定理3

**如果 $n$ 是质数, 那么 AKS 算法会输出"质数"**

显然, 若 $n$ 为质数, 那么在整个算法的第1步、第3步不可能返回. 而如果存在  $a$  使得 不满足第5步的判别式, 那么由引理1可以证明 $n$ 必然不是质数,  因此只要 $a$ 是质数, 在第5步也一定不可能返回. 因此如果 $n$ 是质数, 那么 AKS 算法输出的会是"质数". 

### 定理4

**如果 AKS 算法输出的是"质数", 那么 $n$ 是质数**

首先, 如果 AKS 在第 4 步返回"质数", 那么意味着 $n \leq r$ 并且 $\nexists a \leq r$ 使得 $gcd (a, n)  = 1 \ or \ n$ 那么显然有 $n$ 为质数.  接下来考虑在第 6 步返回质数的情况. 

> 大致思路如下:  1. 首先我们考虑证明第2步的  $r$  是确实存在的, 并且对第2步所找到的 $r$ 限制一个上界.  2. 然后我们证明在满足该上界的  $r$  的下, 只需要对第5步中所示的  $a$  的范围进行验证, 即可完全证明 $n$ 是一个单一质数的幂次.  3. 由于在第一步中已经排除了 $n$ 是个质数的 $k \  (k>1)$ 次幂的可能性, 因此  $n$  只能是质数. 

#### 引理3

**满足 $ord_{r} (n)  > log^{2}n$ 的 $r$ 是存在的, 并且至少存在一个满足条件的 $r$ 是不大于 $max\{3, \lceil log^{5}n \rceil\}$ 的**

**证明:**  

当 * $n=2$ * 时, 取 *$r=3$* 即可.  

当  $n>2$  时, 考虑所有满足 $ord_{r} (n)  \leq \log n$ 或者 $r|n$ 的 $r$  (不妨记为 $\{r_1, r_2, ..., r_t\}$ ) , 那么显然有: 

$$r|n*\Pi_{i=1}^{\lfloor \log^2n\rfloor} (n^i-1)  \quad  (\forall r \in \{r_1, r_2, ..., r_t\})$$ 

即可得到: 

$$lcm (\{r_1, r_2, ..., r_t\}) |n*\Pi_{i=1}^{\lfloor \log^2n\rfloor} (n^i-1)$$

对该式子简单放缩即可得到: 

$$\begin{aligned} n*\Pi_{i=1}^{\lfloor \log^2n\rfloor} (n^i-1)  &< n*\Pi_{i=1}^{\lfloor \log^2n\rfloor}n^i \\ &=n^{1+ (1+\lfloor \log^2n \rfloor) \lfloor \log^2n \rfloor/2} \\ &<n^{\log^4n} \\ &= (2^{\log n}) ^{\log^4 n} \\ &=2^{\log^5n} \end{aligned}$$

即是说:  $lcm (\{r_1, r_2, ..., r_t\})  < 2^{\log^5n}$ . 而我们又由引理2知道,  $LCM (\lceil\log^5n\rceil)  \geq 2^{\lceil\log^5n\rceil}$ . 因此我们就有, 至少存在一个不大于 $\lceil\log^5n\rceil$ 的数 $s$ ,  $s\notin \{r_1, r_2, ..., r_t\}$ . 接下来只需要证明, 存在一个 $s$ ,  $ord_{s} (n) $ 是存在的即可. 因为 $ord_{s} (n) $ 存在当且仅当 $\gcd (s, n) =1$ , 因此只需要考虑 $s$ 和  $n$  不互质的情况. 事实上, 如果  $s$  和  $n$  不是互质的, 那么我们只需要取 $s' = \frac{s}{\gcd (s, n) }$ , 由于 $gcd (s, n)  \in \{r_1, r_2, ..., r_t\}$ 而 $s \notin \{r_1, r_2, ..., r_t\}$ , 因此 $s' \notin \{r_1, r_2, ..., r_t\}$ , 且 $s'$ 与  $n$  是互质的, 于是 $s'$ 即为我们要找的那个 $r$ , 原命题得证. 

$\blacksquare$

#### 引理4

**对 $a \in [1, \lfloor \sqrt{\phi (r) }\log{n} \rfloor]$ 如果 $(X + a) ^n =  (X^n + a)  \  (mod \ X^r - 1,  n)$ 均成立, 那么 $n$ 为质数**

**证明**:

首先, 我们考虑定义如下: 

对于一个多项式函数 $f(X)$ 和一个整数 $m$, 如果有 $$[f (X) ]^m = f (X^m)  \ \  (mod \ X^r -1,  p)$$ 那么我们称 "$m$ 对  $f (X)$  是内省的". 

容易证明 "内省" 的两个性质:  

- *如果 *$m$* 和 * $m'$ * 对 * $f(x)$ * 是内省的, 那么 * $m*m'$ * 对 * $f (x)$ * 也是内省的.  * 
- 如果 $m$ 对 $f(x)$ 和  $g (x)$  都是内省的, 那么  $m$  对  $f(x)*g (x)$  也是内省的. 

为了方便, 接下来我们将记 $l$ 为 $\lfloor \sqrt{\phi (r) }\log{n} \rfloor$ . 

由于  $ord_r (n) >1$ , 所以至少存在一个 $p|n$ , 使得 $ord_r (p) >1$ , 并且显然 $p>r$  (否则在第 3 或第 4 步就会返回) . 

由于我们有: 

$(X+a) ^n = X^n+a \ \  (mod \ X^r-1, n)  \quad  (\forall a \in [0,  l])$ 

于是我们显然有: 

$(X+a) ^n = X^n+a \ \  (mod \ X^r-1, p)  \quad  (\forall a \in [0,  l])$ 

又因为  $p$  是质数, 因此由引理 1 我们有: 

$(X+a) ^p = X^p+a \ \  (mod \ X^r-1, p)  \quad  (\forall a \in [0,  l])$ 

根据上面两个结果我们有: 

$$(X + a) ^{n} =  ( (X + a) ^{n/p}) ^{p} =  (X+a) ^{n/p} \ \  (mod \ X^r-1, p) \\ X^{n/p}+a =  (X^{n/p}+a) ^p = X^n + a \ \  (mod \ X^r-1, p)$$

由此我们可以得到: 

$$(X+a) ^{n/p} = X^{n/p}+a \ \  (mod \ X^r-1, p)  \quad  (\forall a \in [0,  l])$$

于是我们得到了: $n$ 和 $n/p$ 对 $f (X) =X+a$ 都是自省的. ( $a \in [0, l]$ ) 

再由于自省的两个性质, 我们有: 对于集合 $I=\{ (\frac{n}{p}) ^{i}*p^j|i, j\geq0\}$ 中的任意元素, 他们对集合 $P=\{\prod_{a=0}^{l} (X+a) ^{e_a}|e_a \geq 0 \}$ 中的多项式均是自省的

接下来我们考虑两个群: 

- $G$: $I$  中元素对  $r$  取模的余数构成的模 $r$ 意义下的乘群
- *显然有 * $G$ * 是 *$\mathbb{Z}_{r}^{*}$* 的子群 *
- 记  $|G| = t$  , 那么 $t \geq ord_{r} (n)  > log^{2}n$ 
- $\mathcal{G}$ :  考虑 $r$ 阶分圆多项式 $Q_r (X)$ , 在 $F_p[X]$ 中能被分解为 $ord_{r} (p)  > 1$ 个次数为 $\frac{\phi (n) }{ord_{r} (p) }$ 的不可约多项式. 记其中一个为  $h (X)$ . 容易验证,  $P$  中元素对  $h (X)$  和  $p$  取模构成一个群. 即: 由 $X, X+1, X+2, ..., X+l$ 在域  $F_p[x]/ (h (x) )$  中生成的元素按照乘法构成的乘法群.  
- $\mathcal{G}$  是 $F_p[x]/ (h (x) )$ 中乘法群的子群

接下来我们对 $\mathcal{G}$ 的大小做估计, 并进而引出矛盾, 以证明  $n$  必为质数. 

**命题1:**  $|\mathcal{G}| \geq C_{t+l}^{t-1}$ 

**证明:**  

*首先我们证明: 对于 * $P$ * 中任意两个次数小于 * $t$ * 的不同的多项式 * $f (x)$ * 和 * $g (x)$ * , 在 * $\mathcal{G}$ * 中的像也是不同的.*

*假设在 *$F_p[x]/ (h (x) )$* 中 *$f (x)  = g (x)$* 那么我们任取 *$m \in I$* , 首先显然有 *$ (f (x) ) ^m =  (g (x) )^m$* , 又由于 *$m$* 对 *$f$* 和 * $g$ * 均是内省的, 并且 *$h (x)$* 是 *$X^r-1$* 的因式, 于是我们可以得到: * $f (x^m) =h (x^m)$* , 于是 *$x^m$* 是 * $f-g$ * 的一个根.*

*因为 * $m$ * 是在 * $I$ * 中任意取的, 因此 *$m$* 至少有 * $t$ * 个取值, 即 * $f-g$ * 将会有 * $t$ * 个根, 这与 *$f-g$* 的次数小于 *$t$* 矛盾. 于是我们证明了 *$f$* 和 * $g$ * 的像应该是不同的. *

由于 $l = \lfloor \sqrt{\phi (r) }\log{n} \rfloor < \sqrt{r}\log{n} < r < p$ , 因此 $0, 1, 2, ..., l$ 在 $F_p$ 中均为不同的元素. 因此 $X, X+1, X+2, ..., X+l$ 在 $F_p[x]/ (h (x) )$ 中也均为不同的元素. 同时由于 $h (x)$ 的次数大于 $1$ , 因此 $\forall a \in [0, l]$ ,  $X+a != 0$ , 即 $\mathcal{G}$ 中至少有  $l+1$  个不同的一次多项式.  

考虑 $P$ 中所有  $t-1$  次多项式, 即: 满足 $\sum_{a=0}^{l}e_a = t-1$ 的  $e_a$  组合方案数, 用隔板法可以轻松的得到, 为 $C_{t+l}^{t-1}$ . 于是引理1得证. 

$\blacksquare$

**命题2:** 如果 $n$ 不是某个质数的幂次, 那么 $|\mathcal{G}| \leq n^{\sqrt{t}}$ 

**证明:** 

*我们构造一个 *$I$* 的子集 *$\hat{I}: =\{ (\frac{n}{p}) ^{i}*p^{i}|0 \leq i, j \leq \lfloor\sqrt{t} \rfloor\}$ *. 如果 * $n$ * 不是 * $p$ * 的幂次, 那么 *$\hat{I}$* 应该有 *$(\lfloor\sqrt{t}\rfloor+1) ^2 > t$* 个元素. 又因为 *$|G|=t$ *, 所以根据抽屉原理, * $\hat{I}$* 中至少有两个不同的元素 *$m_1$* 和 * $m_2$ * 在模 * $r$ * 的意义下是相同的. 于是我们有 *$X^{m_1} = X^{m_2} \  (mod \ X^r - 1)$* . *

任取  $f (x)  \in P$  , 结合自省性即可得到:   $(f (x) ) ^{m_1}=f (x^{m_1}) =f (x^{m_2}) = (f (x) ) ^{m_2} \  (mod \ X^r - 1, p)$  又因为 $h (x)$ 是 $X^r-1$ 的因子, 因此在 $F_p[x]/ (h (x) )$ 中,  $(f (X) ) ^{m_1}= (f (X) ) ^{m_2}$. 于是  $f (x)$  是多项式 $Y^{m_1}-Y^{m_2}$ 的一个根. 因为 $max\{m_1, m_2\} \leq  (\frac{n}{p}*p) ^{\lfloor\sqrt{t}\rfloor} \leq n^{\lfloor\sqrt{t}\rfloor}$, 因此该多项式至多只有 $n^{\lfloor\sqrt{t}\rfloor}$ 个根. 又因为 $f (x)  \in \mathcal{G}$ 且  $f (x)$  是任意取的, 因此有 $|\mathcal{G}| \leq n^{\lfloor\sqrt{t}\rfloor}$ 成立. 于是命题2得证. 

$\blacksquare$

接下来我们使用命题1和命题2来证明最终的结论: 

由命题1可知: $|\mathcal{G}| \geq C_{t+l}^{t-1}$ , 通过一些简单的放缩, 我们有: 

$$\begin{aligned} |\mathcal{G}| &\geq C_{t+l}^{t-1} \\ &\geq C_{1+\lfloor\sqrt{t}\log n\rfloor + l}^{\lfloor\sqrt{t}\log n\rfloor} \quad  (t > \sqrt{t} \log n)  \\ &\geq C_{2\lfloor\sqrt{t}\log n\rfloor + 1}^{\lfloor\sqrt{t}\log n\rfloor} \quad  (l = \lfloor\sqrt{\phi (r) }\log n\rfloor \geq \lfloor\sqrt{t}\log n\rfloor)  \\ &>2^{\lfloor \sqrt{t}\log n \rfloor+1} \quad  (\lfloor \sqrt{t}\log n \rfloor > \lfloor \log^2 n \rfloor \geq 1,  C_{2x+1}^{x} > 2^{x+1}  (x>1) )  \\ &\geq  (2^{\log n}) ^{\sqrt{t}} = n^{\sqrt{t}} \end{aligned}$$

这就说明, 如果 $|\mathcal{G}|>n^{\sqrt{t}}$ . 但由命题2可知, 如果 $n$ 不是某个质数的幂次, 那么 $|\mathcal{G}|\leq n^{\sqrt{t}}$ , 这与该结论相矛盾. 因此 $n$ 必为某个质数的幂次. 由于在第一步中已经排除了  $n$  是个质数的 $k \  (k>1)$ 次幂的可能性, 因此 $n$ 只能是质数. $\blacksquare$

至此, 算法正确性得证. $\blacksquare$

***

## 复杂度证明

记 $O (t(n) *poly (\log t (n) ) )$ 为 $O^{\sim} (t(n) )$. 

如: 
- $O^{\sim} (\log^k n)  = O(\log^k n * poly(\log\log n)) =O(\log^{k+\epsilon}n)$
- 两个  $m$  位数的四则运算耗时为  $O^{\sim} (m)$ 
- 两个系数为 $m$ 位数的  $d$  次多项式的四则运算耗时为 $O^{\sim} (d*m)$ 

逐步分析复杂度: 

### 步骤一

*判断 * $n$ * 是否是某个数的整数次幂, 复杂度为 * $O^{\sim} (\log^{3} n)$ * [^2] *

### 步骤二

找到最小的  $r$  使得 $ord_r (n)  > \log^2n$  

由  $r$  的范围可知, 我们至多需要尝试 $O (\log^5n)$ 个  $r$ 

对每个 $r$ , 我们对每个 $k \leq \log^2{n}$ 验证 $n^k = 1  (mod \ r)$ 是否成立

每次验证至多需要做 $O (\log^{2}n)$ 次模  $r$  意义下的乘法, 因此复杂度为 $O^{\sim} (\log^2{n} \log r)$ 

综上, 总复杂度为  $O^{\sim} (\log^{7}n)$ 

### 步骤三

*判断 *$r$* 次 * $\gcd$ *, 复杂度为* $O (r*\log n) =O (\log^{6}n)$ 

### 步骤四

整数加法, 复杂度  $O (\log n)$  

### 步骤五

对 $O (\sqrt{\phi (r) }\log n)$ 个系数长 $O (\log n)$ 的多项式在 $\mathbb{Z}_{n}[X]/ (X^r-1)$ 中做取模操作. 每次操作多项式乘法复杂度 $O^{\sim} (r\log n)$ , 共进行 $O (\log n)$ 次操作, 因此总复杂度为 $O^{\sim} (r\sqrt{\phi (r) } \log^3 n)  = O^{\sim} (r^{3/2} \log^3 n)  = O^{\sim} (\log^{10.5}n)$ 

**综上, AKS 算法的总复杂度为** $O^{\sim} (\log^{10.5}n)$

[^2]: 可以参考 *Modern Computer Algebra*