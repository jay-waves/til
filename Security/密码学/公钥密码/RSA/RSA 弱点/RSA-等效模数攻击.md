## 等效模数攻击

由[费马小定理](../../../../../Math/数论/欧拉定理.md), 得到: $$\begin{cases}m^{(p-1)(q-1)/g}\equiv (m^{p-1})^{(q-1)/g}\equiv1\pmod{p} \\ m^{(p-1)(q-1)/g}\equiv (m^{q-1})^{(p-1)/g}\equiv1\pmod{q} \end{cases}$$, 其中 $g=gcd(\ p-1,\ q-1\ )$.

由[模运算](../../../../../Math/数论/模运算.md)性质五, 得到方程解: $$\large m^{\frac{(p-1)\times(q-1)}{g}}\equiv 1\pmod{p\times q}$$

假设 $d'$ 满足: $$d'\times e\equiv 1\pmod{\frac{(p-1)\times (q-1)}{g}}$$, 写作: $$d'\cdot e=1+k\cdot \frac{(p-1)(q-1)}{g},\quad k\in \mathbb{Z}$$

由此: $$\large m^{d'\times e}\equiv m^{1+k\cdot{(p-1)(q-1)}/{g}}\equiv m\cdot (m^{(p-1)(q-1)/g})^{k}\equiv m\pmod{p\times q}$$

可见, $d'$ 和原私钥是**等效**的, 但值更小, 计算速度更快. 当然, 代价是安全性下降, 因此应避免 $g=gcd(p-1,\ q-1)$ 过大, 防止存在较小等效私钥 $d'$, 缩小密钥空间.