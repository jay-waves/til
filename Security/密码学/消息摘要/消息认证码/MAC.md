
# MAC

**定义: 消息认证码, MAC (message authentication code), 是一种消息认证技术, 是指消息被一密钥控制的公开函数作用后产生的, 用作认证符的, 固定长度的数值. 也被称为密码校验和.**  

消息认证包括*消息完整性认证*以及*信源身份认证*. 防御伪装和篡改行为.  
$MAC=C_{K}(M)$. M 为输入消息, C 为 MAC 函数, K 为*共享密钥*, MAC 为消息认证码. 使用时和原消息, 身份认证符一起发送.

MAC **优势和特点**:  
- 普通消息摘要只能保证完整性, 而 MAC 还能认证消息来源 (真实性)
- [公私钥体系](../../公钥密码/数字签名/数字签名.md)有**不可否认性**; 而 MAC 双方都有密钥, 都可以伪造消息. 
- MAC 计算速度更快, 但不适合大规模分发.

### MAC 实现方式

- 基于[消息摘要](../消息摘要.md), 比如 [HMAC](HMAC.md) / [KMAC](KMAC.md) / [UMAC](UMAC.md)
- 基于分组密码, 比如  CMAC / CBC-MAC / PMAC
- 基于伪随函数

### MAC 安全性要求

1. **抗第二原像攻击**: 指敌手在不知道密钥K情况下, 伪造一个有相同MAC值的消息 $M'$ 在计算上不可行.
2. $C_{k}(M)$ 在值域中分布均匀, 阻止 *基于选择明文的穷举攻击*. 分布均匀, 指随机选消息 $(M, M')$, $Pr[C_{K}(M)=C_{K}(M')]=2^{-n}$. 这样保证, 敌手可以选择明文攻击时, [需平均穷举 $2^{n-1}$次](../生日攻击.md)
3. 认证算法对消息某部分不应比其他部分更弱. $M'=f(M)$, 比如对 M 局部插入几比特, 保证 $Pr[C_{K}(M)=C_{K}(M')]=2^{-n}$

