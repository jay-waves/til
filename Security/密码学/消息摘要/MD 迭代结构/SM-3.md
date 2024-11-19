---
code: src/cryptography/sm3.py
---

**SM3密码杂凑算法**于 2010 年发布, 2012 年发布为密码行业标准 (GM/T 0004-2012), 2016年发布为国家密码杂凑算法标准 (GB/T 32905-2016).

### SM3总体结构

SM3 采用 [Merkle-Damgard 结构](Security/密码学/消息摘要/MD%20迭代结构/MD%20结构.md), **基于 SHA-256**, 安全性和SHA-256相当. 

![|300](attach/密码学_SM3哈希函数.png)

