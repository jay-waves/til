## RSA 参数选择

### 公钥 $e$

*$e$ 通常建议值是: 3, 5, 17, 257,  65537 ($2^{16}+1$)*

1. 二进制数中 `1` 数量少, 平方乘算法快, 加密性能好.
2. 公钥 $e$ 较小, 节省加密时间, 并提高私钥 $d$ 安全性. 由于解密可以用陷门 $p,q$ 加速, 所以总体加解密时间更短.
3. 在恰当随机填充下 (如OAEP), 公钥较小也可以安全. 当然小如 $1, 2$ 也不行.

什么 $e$ 不行?
- $e$ 较小, 如 $e=3$, 且不正确填充 (如 `PKCS#1 v2.1 OAEP`), 存在 [RSA-低指数广播攻击](RSA-攻击/RSA-低指数广播攻击.md), 即 [Coppersmith's attack](https://en.wikipedia.org/wiki/Coppersmith%27s_attack)
- $e$ 使 $e^{i}\equiv 1 \pmod{\phi(n)}, i\geq \frac{\phi(n)}{2}$ 的 $i$ 较小, 存在 [RSA-循环攻击](RSA-攻击/RSA-循环攻击.md)

### 大素数 $p,q$

什么 $p, q$ 不行?
- $p,q$ 差值过小不行, 存在 [RSA-费马攻击](RSA-攻击/RSA-费马攻击.md). 当 N 取 `1024b` 时, pq 可分别取 `516b, 508b`
- $p-1, q-1$ 素因子皆较小, 此时 $\phi(N)$ 的因子也较小, 分解难度被降低. $p,q$ 最好为安全素数, 见 [RSA-循环攻击](RSA-攻击/RSA-循环攻击.md). 应先选择素数 $p', q'$, 然后确定 $p=2\times p'+1, q=2\times q'+1$ 也是素数.
- $p-1, q-1$ 有较大公因子, 由于存在[等效模数的问题](RSA.md), 会存在相对很小的等效私钥 $d$, 被较快枚举破解. $p-1, q-1$ 最大公因子最好为 2, 即不存在任何奇数公因子.

### 私钥 $d$

什么 $d$ 不行?
- $d$ 过小, 存在 [RSA-Wiener攻击](RSA-攻击/RSA-Wiener攻击.md), 在多项式时间破解 $d$. 一般要求 $d\geq \sqrt[4]{n}$

### 模数 $N$

什么 $n$ 不行?
- 多个用户公用一个模数 $N$, 或一个模数 $N$ 生成多组密钥, 存在 [RSA-共模攻击](RSA-攻击/RSA-共模攻击与密钥泄露.md). 即, 杜绝使用同一模数 $N$, 加密同一信息 $m$.
- 同一模数 $N$ 派生多组密钥, 且其中某个私钥 $d$ 泄露. 存在 [RSA-密钥泄露攻击](RSA-攻击/RSA-共模攻击与密钥泄露.md), 此时必须更换新模数 $N$.

### 避免特殊情况

当明文等于密文时, 程序应有提示. 如 $e\equiv log_{m}(\ k*N\ +\ m\ )$ 时??

***

## RSA 密钥生成

1. 选取大素数 $p,q$, 当 $n$ 为 `1024b` 时, 可分别取 `516b, 508b`; $n$ 取 `2048b` 时, 可分别取 `1018b, 1030b`, 拉开差值. 
2. (可选) 检查 $p,q$ 是否是安全素数.
3. 检查 $gcd(p-1,q-1)$ 是否足够小, 最佳为2; 条件2满足时, 此条件一定满足.
4. 检查 $\phi(\phi(n))\geq \frac{\phi(n)}{2}$; 条件2满足时, 此条件一定满足.
6. 计算 $n=p\times q$, 其欧拉函数值为 $\phi(n)=(p-1)(q-1)$
7. 选取 $1<e<\phi(n)$, 满足 $gcd(\phi(n),\ e)=1$. 常用值为 65537.
8. 计算 $d$, 满足 $d\equiv e^{-1} \pmod{\phi(n)}$
9. 尽可能销毁或保密 $p,q,\phi(n)$

另一种方式是先选私钥 $d$, 以保证其安全性质:
1. 选取安全素数 $d$, 满足 $d>p$, $d>q$, 接近 $\phi(n)$.
2. 检查 $gcd(d,\ \phi(n))=1$; 若满足条件1, 此条件一定满足.
3. 计算 $e=d^{-1}\pmod{\phi(n)}$, 满足 $gcd(e,\ d)=1$, 
4. 检验 $e$ 是否过小 (如 1, 2).

## 参考

*<应用密码学>* 2017.P156-157  

 [超重点 RSA 参数选择](http://www.waveshare.net/study/article-700-1.html)    
 
 [RSA 算法中几种可能泄密的参数选择](https://wenku.baidu.com/view/1743d7a6284ac850ad024289.html)    

[RSA 填充方法](https://zhidao.baidu.com/question/1303282736275569219.html)  
  
[PKCS#1Padding](https://blog.csdn.net/jinhill/article/details/6607859)    

[强素数生成方法](https://wenku.baidu.com/view/ac764f573c1ec5da50e27078.html?re=view)  

[Why RSA Decryption takes longer time than Encryption - SO](https://stackoverflow.com/questions/2316241/why-rsa-decryption-process-takes-longer-time-than-the-encryption-process)

https://en.wikipedia.org/wiki/RSA_(cryptosystem)