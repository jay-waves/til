---
date: 24-11-05
url: https://datatracker.ietf.org/doc/html/rfc8017
---

PKCS (Public-Key Cryptography Standards, 公钥密码学标准) 由原 RSA Data Security Inc. 发布的一系列不断更新的标准. PKCS #1 重点规定了 RSA 加密和签名算法, 包括 PSS 签名填充算法 (v2.1, 2002) 和 OAEP 加密填充算法 (v2.0, 1993).

实践中, RSA 加密由于明文长度限制, 必须将数据进行分块. 对于每个明文分组, 其数值 ${} n$ 应满足 $2^{s}<n\leq 2^{s+1}$, 其中 ${} s$ 为安全参数/密钥长度. 如 `2048b` 密钥最多可单次加密 `2048b` 的数据.

为了达到 [CPA](../../可证明安全.md) 安全性并防御某些攻击, 算法对每个明文块进行[填充 (padding)](../../分组密码/填充.md). 

## PKCS#1 v1.5 Padding

该算法是 PKCS#1 早期版本 (v1.5) 提出的经典填充算法, 可防御一些简单攻击[^1], 目前不再安全[^2].

[^1]: 如 [RSA-篡改攻击](RSA-攻击/RSA-篡改攻击.md), [RSA-循环攻击](RSA-攻击/RSA-循环攻击.md), [RSA-低指数广播攻击](RSA-攻击/RSA-低指数广播攻击.md), [RSA-共模攻击与密钥泄露](RSA-攻击/RSA-共模攻击与密钥泄露.md), [RSA-选择密文攻击](RSA-攻击/RSA-选择密文攻击.md), 使 RSA 有简单的 CPA 与 CCA 安全性.

[^2]: 著名的 [Bleichenbacher 攻击 (1998)](https://archiv.infsec.ethz.ch/education/fs08/secsem/Bleichenbacher98.pdf), 证明该填充并不是 CCA 安全的. Bleichenbacher 利用了 PKCS#1 填充程序的不严谨性, 如果程序反馈的错误信息能够看出填充格式是否正确, 就能通过构造[明文移位](RSA-攻击/RSA-篡改攻击.md)来修改和猜测明文的值.

PKCS#1 v1.5 每次. 当用于加密时, 块类型 `BT=0x02`, 同时 `PS` 使用随机非零字节生成; 当用于签名时, `BT=0x01`, 同时 `PS` 完全由重复 `0xff` 组成.

```
rsa encrypt:
       +----+----+-----------------------------------+
 EM =  | 00 | 02 |       PS=RANDOM      | 00 |   M   |
       +----+----+-----------------------------------+


rsa signature:
       +----+----+-----------------------------------+
 EM =  | 00 | 01 |     PS=0xff0xff...   | 00 |   M   |
       +----+----+-----------------------------------+
```

PKCS#1 v1.5 至少需要 11 个字节: 两字节 `0x00`, 一字节 `BT`, 至少八字节 `PS`. **填充后总比特长度应和密钥比特长度相同, `M` 和 `PS` 之间应该只有一个零字节, 首字节 `0x00` 保证了明文数值大小比密钥数值大小更小**. (PKCS#1 v1.5 和 OAEP 都要求这一点)

## OAEP Padding

OAEP (Optimal Asymmetric Encryption Padding) 是现代 RSA 加密的填充方式.


```
                     +----------+------+--+-------+
                DB = |  lHash   |  PS  |01|   M   |
                     +----------+------+--+-------+
                                    |
          +----------+              |
          |   seed   |              |
          +----------+              |
                |                   |
                |------> MGF1 ---> xor
                |                   |
                V                   |
               xor <---- MGF1 <-----|
                |                   |
                V                   V
       +----+------------+----------------------------+
 EM =  | 00 | maskedSeed |          maskedDB          |
       +----+------------+----------------------------+
```

- DB: data block
- lHash: hash of label (context, usually set to emtpy)
- MGF: mask generation function
- PS: padding string


## PSS Padding

常见针对签名的攻击手段: [RSA-篡改攻击](RSA-攻击/RSA-篡改攻击.md). 

PSS 填充引入随机化盐值, 用于签名

```
				 +----------+------+--------+
		    DB = | Padding  | salt | mHash  |
		         +----------+------+--------+
		                          |
		                          | 
		          +---------------+-------------+
		          |                             |
		          |                            hash
		          V                             |
		         xor <-------- MGF1 <---------- |
		          |                             |
		          V                             V
		+----+----------------+--------------------+
   EM =	| 00 |  maskedDB      |        H           |
		+----+----------------+--------------------+

```

- DB: 消息散列构造
	- Padding: `0x00` 填充
	- salt: 随机生成的盐值
	- mHash: 消息的哈希值
- MGF: 掩码生成函数

## 代码实现

[教科书式 RSA Python 实现](../../../../src/密码学算法/RSA.py.md).

[OAEP Python 实现](../../../../src/密码学算法/OAEP.py.md)