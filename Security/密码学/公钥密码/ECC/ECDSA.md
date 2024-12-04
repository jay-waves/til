## ECDSA

椭圆曲线签名算法 (ECDSA, Elliptic Curve Digital Signature Algorithm)[^1] 

[^1]: "Public Key Cryptography for the Financial Services Industry: The Elliptic Curve Digital Signature Algorithm (ECDSA)", X9.62-1998, ANSI approval 7 January 1999.

设私钥 $d$, 公钥 $P=[d]G$, 基点 G

### 签名

私钥签名:  

- 选择随机数 $k$, 计算 $[k]G$. 其中 $k\in [1,\ n-1]$, n 为 G 的阶
- 令 $(x,y)=[k]G$, 若 $x=0\pmod n$, 则重选 $k$
- $s=k^{-1}(hash(M)+dx)$. 其中, M 代表消息. 若 $s=0\pmod n$, 则重选 $k$
- 将消息 M 和 签名 $\{\ x, s\ \}$ 发送

### 验签

公钥验签:

- 检查 $x,\ s\in [1,\ n-1]$
- 计算 $(x',\ y') = s^{-1}([hash(M)]G+[x]P)$
- 验证 $x'\equiv x$

正确性证明: $$\frac{[hash(M)]G+[x]P}{s}=\frac{[hash(M)+xd]G}{s}=[k]G$$

就是将整数有限域上的 [DSA](../数字签名/DSA%20协议.md) 协议挪到椭圆曲线域上.

