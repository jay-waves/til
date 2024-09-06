本文件夹仅收录**密码学**相关部分, 网络空间安全 (Cyber Security) 相关请移步 [Code/Security](obsidian://open?vault=Code&file=Security%2FREADME)

***

## 内容目录

现代密码学分为单钥密码 (对称密码) 和公钥密码 (非对称密码), 单钥密码分为分组密码 (Block Cipher) 和流密码 (Stream Cipher). 公钥密码效率比单钥密码低, 但提供了更强的密码学功能, 被用于密钥交换和数字签名. 

1. [分组密码](分组密码/分组密码.md)
	- 古典密码
	- [Feistel](分组密码/Feistel-结构/Feistel.md)
	- [SP 结构](分组密码/SP-结构/代换置换网络.md)
	- [链接模式](分组密码/链接模式.md)
1. 公钥密码
	- [ECC](公钥密码/ECC/ECC.md)
	- [RSA](公钥密码/RSA/RSA.md)
	- [数字签名](公钥密码/数字签名.md)
	- [DiffieHellman-密钥交换](公钥密码/DiffieHellman-密钥交换.md)
1. [消息摘要(密码学哈希函数)](消息摘要/消息摘要.md)
	- [MAC](消息摘要/消息认证码/MAC.md)
	- [迭代型散列函数](消息摘要/迭代型散列函数.md)
	- [生日攻击](消息摘要/生日攻击.md)
1. [流密码与伪随机数](流密码与伪随机数/流密码.md)
2. [安全协议](安全协议/安全协议.md)
	- [基于加密的认证协议](安全协议/认证协议/基于加密的认证协议.md)
	- [基于公钥的认证密钥协商](安全协议/认证的密钥协商协议/基于公钥的认证密钥协商.md)
	- [BAN逻辑](安全协议/BAN逻辑.md)
	- [密钥分发与管理](安全协议/密钥分发与管理.md)
1. 可证明安全
	- [Reduction](附录/可证明安全/Reduction.md)
	- [CPA-Secure](附录/可证明安全/CPA-Secure.md)
	- [CCA-Secure](附录/可证明安全/CCA-Secure.md)
	- [Perfect Secrecy](附录/可证明安全/Perfect%20Secrecy.md)
