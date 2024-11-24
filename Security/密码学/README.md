## 目录

现代加密算法分为单钥密码 (对称密码) 和公钥密码 (非对称密码), 单钥密码分为分组密码 (Block Cipher) 和流密码 (Stream Cipher). 

公钥密码效率比单钥密码低, 但提供了更多安全功能, 被用于密钥交换和数字签名. 

1. [ReadMe](分组密码/ReadMe.md)
	- [古典密码](分组密码/古典密码.md)
	- [Feistel](分组密码/Feistel-结构/Feistel.md)
	- [SP 结构](分组密码/SP-结构/代换置换网络.md)
	- [链接模式](分组密码/链接模式.md)
1. 公钥密码
	- [ECC](公钥密码/ECC/ECC.md)
	- [RSA](公钥密码/RSA/RSA.md)
	- [常见数字签名算法](Security/密码学/公钥密码/数字签名/数字签名.md)
	- [DiffieHellman 协议](Security/密码学/公钥密码/DiffieHellman%20协议.md)
1. [消息摘要 (密码学哈希函数)](消息摘要/ReadMe.md)
	- [MAC](消息摘要/消息认证码/MAC.md)
	- [HMAC](消息摘要/消息认证码/HMAC.md)
	- [MD 结构](Security/密码学/消息摘要/MD%20迭代结构/MD%20结构.md)
	- [生日攻击](消息摘要/生日攻击.md)
	- [MD-5](Security/密码学/消息摘要/MD%20迭代结构/MD-5.md)
	- SHA-1
1. [流密码与伪随机数](流密码与伪随机数/ReadMe.md)
	- [流密码](流密码与伪随机数/ReadMe.md)
	- [RC4](流密码与伪随机数/流密码算法/RC4.md)
2. [安全协议](安全协议/ReadMe.md)
	- [基于单钥的认证密钥协商](安全协议/认证的密钥协商协议/基于单钥的认证密钥协商.md)
	- [基于公钥的认证密钥协商](安全协议/认证的密钥协商协议/基于公钥的认证密钥协商.md)
	- [AKA 协议](安全协议/认证的密钥协商协议/AKA%20协议.md)
	- [Kerberos 协议](安全协议/认证的密钥协商协议/Kerberos%20协议.md)
	- [密钥分发与管理](安全协议/密钥分发与管理.md)

## 密码性能对比

主要测试 Intel 酷睿平台下, OpenSSL 软件的各类密码算法性能.

| 算法                                                   | 类型     | 测试时间 | 吞吐量 (MiB/S)[^1]                                              | OpenSSL 版本 | 平台                                    |
| ------------------------------------------------------ | -------- | -------- | --------------------------------------------------------------- | ------------ | --------------------------------------- |
| [新版本多核测试](https://openbenchmarking.org/suite/pts/cryptography)[^4]                                                        |          |          |                                                                 |              |                                         |
| AES-128-GCM                                            | 块密码   | 2024     | 150,106                                                         | 3.3.x        | Intel Core i9-14900K, 5.7GHz, 24c, 32t  |
| [AES](Security/密码学/分组密码/SP-结构/AES.md)-256-GCM | 块密码   |          | 131,518                                                         |              |                                         |
| SHA256                                                 | 哈希     |          | 31,903                                                          |              |                                         |
| SHA512                                                 | 哈希     |          | 9,919                                                           |              |                                         |
| ChaCha20-[Poly1305](Security/密码学/消息摘要/消息认证码/UMAC.md)                                      | 流密码   |          | 41,961                                                          |              |                                         |
| RSA4096                                                | 公钥密码 |          | <nobr>331,528 (verify/s)</nobr>[^3] <nobr>5,144 (sign/s)</nobr> |              |                                         |
| [单机单核测试](https://openwrt.org/docs/guide-user/perf_and_log/benchmark.openssl)                                                       |          |          |                                                                 |              |                                         |
| [MD5](Security/密码学/消息摘要/MD%20迭代结构/MD-5.md)                                                    | 哈希     | 2013     | 501                                                             | 1.1.x        | Intel Core i7 4960X, 3.6GHz, 4c, 8t|
| SHA1                                                   | 哈希     |          | 541                                                             |              |                                         |
| SHA256                                                 | 哈希     |          | 217                                                             |              |                                         |
| SHA512                                                 | 哈希     |          | 317                                                             |              |                                         |
| [DES](Security/密码学/分组密码/Feistel-结构/DES.md)                                                    | 块密码   |          | 75.73                                                           |              |                                         |
| [3DES](Security/密码学/分组密码/Feistel-结构/EDE.md)                                                   | 块密码   |          | 28.24                                                           |              |                                         |
| AES-128-CBC, 192, 256                                  | 块加密   |          | 116.86, 109.87, 89.92                                           |              |                                         |
| [RSA2048](Security/密码学/公钥密码/RSA/RSA.md)                                                | 公钥密码 |          | 23847.8 (verify/s) 747.7 (sign/s)                               |              |                                         |
| [DSA](Security/密码学/公钥密码/数字签名/数字签名.md)2048                                                | 公钥密码 |          | 2111.3 (verify/s) 2438.4 (sign/s)                               |              |                                         |

[^1]: 吞吐量 = 周转速度 (C/S) * 单次数据块大小. 

[^3]: RSA4096 吞吐量大概是: $4096\times 331528 \div 1024\div 1024 =1295\ (MiB/s)$

[^4]: 在 i9-14900K 测试中, 完全开启了硬件并行和各种优化, 比常规测速快很多. 在 i7-4960X 测试中, 只有一个核被使用 (默认情况), 使用 OpenSSl 自带的 Benchmark 工具 `openssl speed`.

对比哈希算法和对称密码 (尤其是 [AES](Security/密码学/分组密码/SP-结构/AES.md)). 如下图, 单次输入数据块的体积很小时 (<1KB), 由于频繁的初始化 (上下文, IO, 填充等开销), 哈希算法吞吐量 (MB/s) 比不上同规模的块密码; 当数据规模逐渐上升, 初始化开销相对于哈希中压缩函数的开销占比减小时, 哈希算法逐渐接近理论上限, 吞吐性能比块密码 (图中使用的 AES-CBC) 更好. 

![|700](attach/throughput%20aes%20vs%20sha256.png)

注意, AES 在开启支持并行的[流模式](Security/密码学/分组密码/链接模式.md) (如 CTR, GCM, OFB) 后, 处理大规模数据时吞吐量并不弱于哈希算法. 哈希算法无法并行, 必须遍历整个输入进行迭代; AES-GCM 可通过硬件 (GPU, 或特殊 AES-NI 指令集) 实现并行加速, 实际大规模吞吐量优于哈希算法[^5]. 

[^5]: 见上表 Intel Core i9-14900K 平台的测试数据.

由于时间侧信道攻击的存在, 激进的优化可能并不可取, 算法的时间消耗需要更加稳定.

## 参考

Introduction to Modern Cryptography. Jonathan Katz.

Cryptograhpy and Network Security -- Principles and Practice. William Stallings. 8th ed.

信息网络安全. 刘建伟. 3rd ed.

[现代密码学简介](https://github.com/Evian-Zhang/Introduction-to-modern-cryptography). Evian-Zhang.