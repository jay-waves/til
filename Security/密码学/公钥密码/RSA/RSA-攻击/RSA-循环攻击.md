---
revised: 24-12-03
---

### 安全素数

RSA 定义**安全素数**为:
- $p$ 为素数
- $\frac{p-1}{2}$ 也为素数

若 $p$ 为安全素数, 则其欧拉函数值 $\phi(p)=p-1$ 只有两个素因子: $2$, $\frac{p-1}{2}$. 

定义安全素数目的, 就是让 $p-1$ 的因子尽可能大 (少). 否则若 $p-1$ 的每个因子都较小, 存在 [Pollard's p-1 算法](https://en.wikipedia.org/wiki/Pollard%27s_p_%E2%88%92_1_algorithm)可以更高效解决离散对数问题. RSA 的安全性也受离散对数问题的影响, 见 [RSA 安全分析](../RSA.md)

当 RSA 的 $p,q$ 皆为素数时, 有: $\phi(N)=\phi((p-1)\cdot(q-1))=\phi(2^{2})\cdot\phi\left( \frac{p-1}{2} \right)\cdot\phi\left( \frac{q-1}{2} \right)=2\cdot\frac{p-3}{2}\cdot \frac{q-3}{2}$. 

### 强素数

RSA 定义**强素数为**:
- $p$ 是素数
- $p-1$ 有大素数因子, 称为 $r$
- $p+1$ 有大素数因子
- $r-1$ 仍有大素数因子

> 强素数的定义也见: <https://en.wikipedia.org/wiki/Pollard%27s_p_%E2%88%92_1_algorithm>, 需要进一步整理, (??存疑??).

... 由[欧拉函数](/Math/数论/欧拉函数.md)知, $\phi(k)=\prod^{s}_{i=1}\left( p_{i}-1 \right)$, 其中 $p_{i}$ 是 $k$ 的素因子. $\phi(p)=p-1$ 有素因子 $r$, 那么 $\phi(p-1)$ 中就有因子 $r-1$. 出于类似的原因, 也让 $r-1$ 存在较大因子.

### 循环攻击

假设攻击者可实施选择明文攻击 (废话), 其截获密文 $c$ 后, 进行重复加密:

$$\begin{align}
c^{e}\equiv m^{e^{2}} \pmod{n}\\
c^{e^{2}}\equiv m^{e^{3}}\pmod{n} \\
\dots \\
c^{e^{t-1}}\equiv m^{e^{t}}\pmod{n} \\
c^{e^{t}}\equiv m^{e^{t+1}}\pmod{n}
\end{align}$$

若 $m^{e^{t}}\equiv m\pmod{n}$, 那么由 $c^{e^{t-1}}\equiv m^{e^{t}}\equiv m\pmod{n}$, 恢复出明文.

此时, $t$ 是满足 $e^{t}\equiv 1\pmod{\phi(n)}$ 的最小数, $t$ 就是公钥 $e$ 的模 $\phi(n)$ 阶 $o_{\phi(n)}(e)$. 由欧拉定理, $t\ \vert\ \phi(\phi(n))$, 即 $t\vert \phi(\ (p-1)\cdot (q-1)\ )$.

要降低该攻击的威胁, 就要增大 $t$. 又因为 $t$ 一定是 $\phi(\ (p-1)\cdot (q-1)\ )$ 的某个因子, 所以需要 $\phi(\ (p-1)\cdot (q-1)\ )$ 的所有素因子更大, 来保证 $t$ 的最小下界. 结合上文可知, 我们既需要 $p-1$ 的素因子尽可能大 (除了平凡素因子 $2$), 又需要 $\phi(p-1)$ 的素因子也尽可能大, 防止用 [Pollard's p-1 算法](https://en.wikipedia.org/wiki/Pollard%27s_p_%E2%88%92_1_algorithm)在指数环上高效地求解 $t\pmod{\phi(\phi(n))}$. (?? 存疑 ??)
