> **WE STAND TODAY on the brink of a revolution in cryptography.**  
> -- [Diffie, W.; Hellman, M. -- New directions in cryptography](https://drive.google.com/open?id=1ROHVZjHb6rskYYETAOaYdIpj7cqmszsX)

DH 算法是第一个公开的非对称加密算法 (1976), 但只被设计用于密钥交换, 没有实现加密.   
DH 算法安全性基于**[有限域离散对数困难问题](../../../math/numth/欧拉定理.md)**

## Diffie Hellman KEP

Alice 和 Bob 协商密钥, 但信道并不安全  

1. Alice 选择随机数 $x$,  计算 $X\equiv\alpha^{x}\pmod p$, 并将 ${} (X\text{ , }\alpha\text{ , p}) {}$ 公开
2. Bob 同样执行上述操作, 选择 $y$, 公开 ${} Y\equiv\alpha^{y}\pmod p {}$
3. Alice 计算 $k=Y^{x}\equiv\alpha^{xy}\pmod p$, Bob 同, 两人获得相同协商值

信道中敌手知 ${} \{X, Y\} {}$ , 但由于离散对数问题困难性, 无法求解 $k$

DH 协议容易扩展为多方, 只需要顺序交换公钥 $\alpha^{x}$ 即可.

## 中间人攻击

中间人 Eve, 通过**截获与欺骗**方式, 与 Alice 和 Bob 分别通讯  

- 截获: Eve 截获 Alice/Bob 发送给对方的公钥信息
- 欺骗: Eve 与 Alice/Bob 分别建立起共同密钥, 但 Alice/Bob 错认为正在与对方 (而不是 Eve) 通信.

| 步骤 | Alice      | 信道                        | Eve         | 信道                        | Bob         |
| ---- | ---------- | --------------------------- | ----------- | --------------------------- | ----------- |
| 1    | $x$        | $g^{x}\to$                  |             |                             |             |
| 2    |            | $\leftarrow g^{z}$          |         |                 |             |
| 3    |            |                             | $z'$        | $g^{z'}\to$                            |             |
| 3    |            |                             |             | $\leftarrow g^{y}$          | $y$          |
| 4    | $K=g^{xz}$ | ${} \longleftrightarrow {}$ | $K=g^{xz}$  |                             |             |
| 65   |            |                             | ${} K'=g^{z'y} {}$ | ${} \longleftrightarrow {}$ | $K'=g^{z'y}$ |

**为了防止中间人攻击, 需要引入基于数字证书的认证**, 改进方案如[有认证性的 MTI 协议](../安全协议/认证的密钥协商协议/基于公钥的认证密钥协商.md).
