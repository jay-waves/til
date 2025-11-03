Rabin 密码体制, 又称平方乘密码, 破译难度和分解大整数等价.

#### 密钥生成

选择大素数 $p,\ q$, 满足 $p\equiv q\equiv 3\pmod 4$, 并计算 $n=p\times q$.

公钥为 $n$, 私钥为 $q,\ p$

#### 加密

$c=m^{2}\pmod{n}$

#### 解密

解 $x^{2}\equiv c\pmod n$, 等价于解:
$$\begin{cases}
x^{2}\equiv c\pmod p \\
x^{2}\equiv c\pmod{q} 
\end{cases}$$

由 $p\equiv 3\pmod 4$, 知 $4\ \vert\ (p+1)$.

由 $c$ 是模 $p$ 的[二次剩余](../../../数学/数论/二次剩余.md), 知 $\left( \frac{c}{p} \right)\equiv c^{\frac{p-1}{2}}\equiv 1\pmod p$.

由 $( c^{\frac{p+1}{4}} )^{2}\equiv ( m^{\frac{p+1}{2}} )^{2}\equiv (m^{2})^{\frac{p+1}{2}}\equiv c^{\frac{p-1}{2}}\cdot c\equiv c\pmod p$, 知:  
$c^{\frac{p+1}{4}}$ 和 $p-c^{\frac{p+1}{4}}$ 是方程 $x^{2}\equiv c\pmod p$ 的两根. 对 $q$ 同理.

所以方程可解出四组同余方程:  

$\begin{cases} x\equiv y\pmod p\\ x\equiv z\pmod q\end{cases}$, $\begin{cases} x\equiv -y\pmod p\\ x\equiv z\pmod q\end{cases}$, $\begin{cases} x\equiv -y\pmod p\\ x\equiv -z\pmod q\end{cases}$, $\begin{cases} x\equiv y\pmod p\\ x\equiv -z\pmod q\end{cases}$

每组皆可用[中国剩余定理](../../../数学/数论/中国剩余定理.md)求解. 为确定有效明文, $m$ 中常加入某些协商信息, 如日期和ID.