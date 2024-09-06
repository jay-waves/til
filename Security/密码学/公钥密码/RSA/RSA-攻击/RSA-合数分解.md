> 该方法说明, 在已知私钥时 可高效==还原出 RSA 素数 $p,q$==.  
> 同时, 也揭示 ==RSA 的本质安全性原理==.

观察参数: $n = p\times q$

和欧拉函数: 

$\begin{split}\phi(n)\ &=\ \phi(p\times q)\\&=\phi(p)\times\phi(q)\\&=(p-1)\times (q-1)\\&=p\times q-q-p+1\end{split}$

, 有: $n-(p+q)+1=\phi(n)$ 

故由韦达定理知, $p$ 和 $q$ 为方程 $x^2-(p+q)*x+pq=0$ 的两个解, 其中 $p*q=n$ , $p+q=n-\phi(n)+1$

又因为: $d\ \times\ e\ \equiv\ 1\pmod{\phi(n)}$ ,有: $d\ \times\ e\ -1 =\ k*\phi(n)$

**如果能从 $k*\phi(n)$ 求出 $\phi(n)$, 就能解出 $p,q$**.  

因为 $n-\phi(n)=p+q-1$, 所以 $p,q$ 数量级相对于 $n$ 和 $\phi(n)$ 较小, 所以可以近似为 $1\approx\frac{\phi(n)}{n}$. 因此 $k\approx \frac{d*e-1}{n}=\frac{k*\phi(n)}{n}$. $k$ 较小, 基本不会有误差. 

2010 年建议的中 $p$ 和 $q$ 在 $2^{512}$ 数量级, $n$ 在 $2^{1024}$ 以上. 如果该法结果偏小, 说明 $\frac{k*(p+q)}{n}>1$, 由于 $d,e$ 都是在有限域上计算的, 这个 $k$ 不会很大. 

## RSA 安全性

在RSA体系中, 公钥 $(e, n)$ 公开, 私钥 $(d, n)$ 保密. 保证RSA安全的, 其实是**保证敌手由 $n$ 求不出 $\phi(n)$**. 而为保证敌手求不出 $\phi(n)$, 则须保证敌手**无法求出 $n$ 的分解 $p\times q$**.

如果敌手由 $n$ 能求出 $\phi(n)$, 那么就能由 $e$ 求出私钥 $d$, 因为 $d\equiv e^{-1}\pmod{\phi(n)}$.

而 $\phi(n)=p*q-q-p+1$, $n=p*q$, 所以两者的差只有 $\phi(n)-n=1-(p+q)$. 设想如果 $p,q$ 过小, 那么敌手就易从 $n$ 遍历出 $\phi(n)$.
