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

由[欧拉函数](../../../../../Math/数论/欧拉函数.md)知, $\phi(m)=m\cdot\prod^{s}_{i=1}\left( 1-\frac{1}{p_{i}} \right)$  , 此时 $m=(p-1)\cdot(q-1)$. 由于 $p, q$ 皆为素数, 所以 $p-1, q-1$ 皆有因子2. 要降低循环攻击的威胁, 就需要增大 $t$, 就需要 $\phi((p-1)\cdot(q-1))$ 更大, 就需要 $(p-1)\cdot(q-1)$ 有较大素因子. ==(不严谨的推导)==

### 安全素数

RSA定义**安全素数**为:
- $p$ 为素数
- $\frac{p-1}{2}$ 也为素数

若 $p$ 为安全素数, 则其只有两个素因子, 且 $\frac{p-1}{2}$ 较大. $\phi((p-1)\cdot(q-1))=\phi(2^{2})\cdot\phi\left( \frac{p-1}{2} \right)\cdot\phi\left( \frac{q-1}{2} \right)=2\cdot\frac{p-3}{2}\cdot \frac{q-3}{2}$.

定义安全素数目的, 就是让 $p-1$ 的因子尽可能大 (少). 否则若 $p-1$ 的每个因子都较小, 存在 Pohlig-Hellman 算法可以更高效解决离散对数问题, 存在 [Pollard's p-1 算法](https://en.wikipedia.org/wiki/Pollard%27s_p_%E2%88%92_1_algorithm). RSA 的安全性也受离散对数问题的影响, 见[RSA安全分析](../RSA.md)


### 强素数

RSA定义**强素数为**:
- $p$ 是素数
- $p-1$ 有大素数因子, 称为 r
- $p+1$ 有大素数因子
- $r-1$ 仍有大素数因子

若 $p,q$ 为强素数, 则 $\frac{p-3}{2}=\frac{p-1}{2}-1$ 也有较大素因子. 使t较大.

==强素数定义哪来的, 没找到==