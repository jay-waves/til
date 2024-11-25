
## 椭圆曲线密码学 ECC

ECC 是建立在基于 [**椭圆曲线上的离散对数困难问题**](../../../../Math/抽象代数/椭圆曲线/椭圆曲线.md) 上的密码体制,  
单陷门函数为: $P=[d]G$. 换言之, 给定基点 G 和公钥 P, 求出私钥 d 是困难的.

> 选择的基点 G 能够保证满足 $[n]G=O$ 的阶 n 足够大, 这样能保证有足够多的私钥.  
> 由于公钥算法 $P=[d]G$, 选取私钥 $d$ 须满足 $1<d<n$, 否则密钥会存在碰撞

### ECC 公钥加密

常见 ECC 加密算法为 [ECC-Elgamal](Security/密码学/公钥密码/ECC/ECC-Elgamal.md), 基于 [ElGamal 协议](Security/密码学/公钥密码/ElGamal%20协议.md), 两者同属于循环群上的离散对数困难问题.

### ECC 密钥交换

ECDH (Elliptic Curve Diffie-Hellman Protocol) 基于 ECC 实现的 DH 密钥交换体系.  

公开信道获取 $[d_{B}]G$, 私钥 $d_{A}$, 获得共享密钥 $[d_{B}\ d_{A}]G$

### ECC 数字签名

椭圆曲线签名算法 (ECDSA, Elliptic Curve Digital Signature Algorithm)[^1] 

[^1]: "Public Key Cryptography for the Financial Services Industry: The Elliptic Curve Digital Signature Algorithm (ECDSA)", X9.62-1998, ANSI approval 7 January 1999.

设私钥 $d$, 公钥 $P=[d]G$, 基点 G

私钥签名:  

- 选择随机数 $k$, 计算 $[k]G$. 其中 $k\in [1,\ n-1]$, n 为 G 的阶
- 令 $(x,y)=[k]G$, 若 $x=0\pmod n$, 则重选 $k$
- $s=k^{-1}(hash(M)+dx)$. 其中, M 代表消息. 若 $s=0\pmod n$, 则重选 $k$
- 将消息 M 和 签名 $\{\ x, s\ \}$ 发送

公钥验签:

- 检查 $x,\ s\in [1,\ n-1]$
- 计算 $(x',\ y') = s^{-1}([hash(M)]G+[x]P)$
- 验证 $x'\equiv x$

原理: $\frac{[hash(M)]G+[x]P}{s}=\frac{[hash(M)+xd]G}{s}=[k]G$

> 分析见 [ElGamal 协议](Security/密码学/公钥密码/ElGamal%20协议.md). SM2 验证方式类似 ElGamal; ECDSA 构造 s 方式和 ElGamal 类似, 但由于签名中已无法恢复出 $[k]G$ (而且椭圆点不能作为标量), 所以采用了 Schnorr 协议的验证 $k$ (而不是 m) 的方式.

### 与 RSA 对比

160/256 位的 ECC 安全性和 1024/2048 位的 RSA 安全性相当. 所以 ECC 计算量效率更高, 同时占用的存储空间和带宽较少.

ECC 缺点是相对较新, 应用和部署不如 RSA 广泛, 某些环境中 ECC 兼容性可能不佳;  
另外 ECC 计算和实现相对 RSA 复杂, 其具体性能依赖于实现方法. 

而 RSA 作为最早的公钥加密算法之一, 被广泛使用和研究, 使用范围更广泛.  
由于公钥加密算法更多用于密钥交换环节, 直接用于加密的比较少, 所以 RSA 比 ECC 消耗更多资源通常是可接受的.
