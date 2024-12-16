## 共模攻击

*使用相同模数生成多组密钥 $e$ 时, 加密的同一信息 $m$ 不安全.* 

### 原理

$$c1 = m^{e1}\ (mod\ N)$$

$$c2 = m^{e2}\ (mod\ N)$$

若两个公钥 ${} e_{1},e_{2} {}$ 互素，根据扩展的欧几里得算法, 存在 $x,y$ 有:

$$e1 \cdot x+ e2 \cdot y\ =\ gcd(e1,\ e2)\ =\ 1$$

可得：

$$
\begin{aligned}
& c_{1}^{x}\cdot c_{2}^{y}\pmod{N}\\
& =(m^{e_{1}}\pmod{N})^{x}\ \cdot\ (m^{e_{2}}\pmod{N})^{y}\pmod{N}\\
&= m^{e_{1}x+e_{2}y}\pmod{N}\\
&= m \pmod{N}
\\
\end{aligned}
$$

也就是在完全不知道私钥的情况下，得到了明文 $m$

$$
m = (c1^{x}\ *\ c2^{y})\pmod{N}
$$

