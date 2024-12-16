
## 密钥泄露

*若某私钥 d 被泄露，则基于该 $(p, q)$ 的所有公私钥对 $(e, d)$ 都会不安全*

建议使用不同 $p,q$ 生成不同的密钥.

### 攻击原理

假设 $p,q$ 生成了两对公私钥对: $(e_{1},d_{1})$, $(e_{2},d_{2})$, 其中 $d_{1}$ 泄露. 攻击手段如下:

1. 在公开信道获取公钥 $e_{1}$, $e_{2}$
2. $e_{1}\times d_{1}\equiv 1\ \pmod{\phi(n)}$, 其中 $\phi(n)=(p-1)\times(q-1)$. 注意仅知道 $d_{1}\cdot e_{1}\equiv k\cdot\phi(n)$, 而不知道 $\phi(n)$.
3. 计算 $d_{2}\equiv e_{2}^{-1}\pmod{e_{1}\times d_{1}-1}\equiv e_{2}^{-1}\pmod{\mathbf{k}\cdot \phi(n)}$. 等价于 $d_{2}\equiv e_{2}^{-1}\pmod{\phi{(n)}}$.

### 模数分解

私钥 $d$ 泄露后, 可以高效还原出参数 $p,q$, 其中模数 $N=p\cdot q$.

由欧拉函数: 

$$\begin{split}\phi(n)\ &=\ \phi(p\times q)\\&=\phi(p)\times\phi(q)\\&=(p-1)\times (q-1)\\&=p\times q-q-p+1\end{split}$$

于是: $n-(p+q)+1=\phi(n)$ 

故由韦达定理知, $p$ 和 $q$ 为方程 $x^2-(p+q)*x+pq=0$ 的两个解, 其中: $p*q=n$ , $p+q=n-\phi(n)+1$.

又因为: $d\ \times\ e\ \equiv\ 1\pmod{\phi(n)}$ ,有: $d\ \times\ e\ -1 =\ k*\phi(n)$

于是 $k=\frac{de-1}{\phi(n)}\approx \frac{de-1}{n}$. 其中 $d$ 为泄露的私钥. 计算误差为 $\epsilon=| \frac{de-1}{n}-\frac{de-1}{\phi(n)}|$. 

在实践中, $e$ 取值较小 (取 $e\leq \sqrt{ \phi(n) }$), $d$ 接近 $\phi(n)$, 因此 $de-1\approx c\cdot \phi^{1.5}(n)$.  
另外, $p,q\approx \sqrt{ n }$, 可知 ${} \epsilon\approx c\sqrt{ \frac{\phi(n)}{n} }=c\sqrt{ \frac{n-(p+q)+1}{n} }\approx c {}$. $c$ 为某常数, 并不大, 可以试探得出.
