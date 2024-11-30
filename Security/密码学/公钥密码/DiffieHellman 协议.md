> **WE STAND TODAY on the brink of a revolution in cryptography.**  
> -- [Diffie, W.; Hellman, M. -- New directions in cryptography](https://drive.google.com/open?id=1ROHVZjHb6rskYYETAOaYdIpj7cqmszsX)

DH 算法是第一个公开的非对称加密算法 (1976), 但只被设计用于密钥交换, 没有实现加密.   
DH 算法安全性基于**[有限域离散对数困难问题](../../../Math/数论/欧拉定理.md)**

## Diffie Hellman KEP

Alice 和 Bob 协商密钥, 但信道并不安全  

1. Alice 选择随机数 $x$,  计算 $X\equiv\alpha^{x}\pmod p$, 并将 ${} (X\text{ , }\alpha\text{ , p}) {}$ 公开
2. Bob 同样执行上述操作, 选择 $y$, 公开 ${} Y\equiv\alpha^{y}\pmod p {}$
3. Alice 计算 $k=Y^{x}\equiv\alpha^{xy}\pmod p$, Bob 同, 两人获得相同协商值

信道中敌手知 ${} \{X, Y\} {}$ , 但由于离散对数问题困难性, 无法求解 $k$

![|400](../../../attach/密码学_DH密钥交换.png)

DH 协议容易扩展为多方, 只需要顺序交换公钥 $\alpha^{x}$ 即可.

## 中间人攻击

中间人 Eve, 通过**截获与欺骗**方式, 与 Alice 和 Bob 分别通讯  

- 截获: Eve 截获 Alice/Bob 发送给对方的公钥信息
- 欺骗: Eve 与 Alice/Bob 分别建立起共同密钥

![|600](../../../attach/密码学_DH中间人攻击.png)

图中 Eve 使用了同一个密钥 $a^z$, 如果 Alice 和 Bob 容易通过这一点识别出攻击者的存在, 不隐蔽. 改进方法 (也是更常用的中间人攻击方式) 是 Eve 使用不同密钥与 Alice/Bob 进行交换.

**为了防止中间人攻击, 需要引入认证**, 比如 A 和 B 互相提供数字签名证书. 改进方案见[有认证性的 MTI 协议](Security/密码学/公钥密码/MTI%20协议.md)
