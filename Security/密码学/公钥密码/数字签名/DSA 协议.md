## DSA 数字签名

DSA 数字签名算法, 是一种基于公钥密码的*带附录数字签名方式*. 用于 [NIST](appx/各类互联网国际标准组织.md) 提出的美国国家标准 [DSS (Digital Signature Standard) 数字签名标准](https://csrc.nist.gov/pubs/fips/186-5/final), DSS 还包括 ECDSA 和 RSA.

DSA 算法是 Elgamal 签名的变种, 同样基于有限域上离散对数问题. 但它比 ElGamal 的效率更高, 签名更短. DSA 和 [ElGamal 签名](Security/密码学/公钥密码/ElGamal%20协议.md) 有一样的安全问题, 即随机数 k 不能重复使用.

DSA 安全性依赖于 $\mathbb{Z}_{p}$ 的 q 阶子群上的 DLP 问题. 这种*大群的小阶子群*的思想来源于 Schnorr.

### 密钥生成

1. 选取素数 $p$, `512b` < 字长 < `1024b`
2. 选取素数 $q$, $q\vert p-1$, `159b` < 字长 < `160b` [^1]
3. 选取有限域 $\mathbb{GF}(p)$ 的 $q$ 阶子群的生成元 $g=h^{(p-1)/q}\pmod{p}$, $h\neq 0$, $h^{(p-1)/q}\neq 1\pmod{p}$ 
4. 选取随机数 $0<x<q$, 作为私钥
5. $(p, q, g)$ 为公钥

[^1]: 24 年推荐标准为 256 位

### 签名

$r=[g^{k} \pmod p] \pmod q$, $k$ 为随机数   
$s=[k^{-1}(Hash(M)+xr)]\pmod q$

### 验签

收到签名 $\{s, r\}$ 与原消息 $M$:

$m=Hash(M)$  
$u_{1}=[m*s^{-1}]\pmod q$  
$u_{2}=r*s^{-1}\pmod q$  
$v=[g^{u_{1}}*y^{u_{2}}\pmod p]\pmod q$

接下来验证 $v\equiv r$, 因为 $v=[g^{[m+xr]*s^{-1}}\pmod p]\pmod q$

### 随机数重用漏洞

...
