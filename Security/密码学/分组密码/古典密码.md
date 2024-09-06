## 古典密码

```
古典密码 -> 代换密码 -> 单字母代换密码 -> 单表代换密码
        |          |                -> 多表代换密码
        |           -> 多字母代换密码
        -> 置换密码 (不常见)

多字母代换密码: Hill 密码, 多字母仿射变换
单表代换密码: 凯撒密码, 移位密码, 放射变换
多表代换密码: Vigenere 密码, Beaufort 密码, Running-Key 密码, Vernam 密码
```

### 单表代换

**凯撒密码** (Caesar Cipher): $C=M+k \pmod{26}$

**采样密码** (Decimation Cipher): $C=M\cdot k\pmod{26}$, $gcd(k, 26)=1$

**仿射密码** (凯撒+采样): $C=M\cdot k_{1}+k_{2}\pmod{26}$

单表替代的密钥空间比较大: $26!$, 但是并不安全. 因为其无法隐藏自然语言统计特征.

### 多表代换 (Polyalphabetic Ciphers)

明文字母表为 $Z_{q}$, 代换表序列: $K=(\sigma_{1}, \sigma_{2},\dots)$, 明文字母: $m=m_{1},m_{2},\dots$. 则密文为: $$C=\sigma_{1}(m_{1}),\sigma_{2}(m_{2}),\dots$$

**如果 K 是非周期无限序列, 则该密码称为一次一密密码 (one-time pad cipher)**.

**Vernam 密码**: 使用与明文等长的密钥流进行异或 (exclusive-or) 加密: $c=k_{i}\oplus m_{i}$. 密钥流足够随机时, 是一次一密.

**Vigenere 密码**: 设明文为 $P=p_{1},p_{2},\dots,p_{d}$, $c_{i}=p_{i}+k_{i}\pmod{26}, i=1,2,\dots ,d$. k 序列可以重复使用, 但建议与明文一样长.

**Hill 密码**: $C=K\cdot P\pmod{26}$, K 为密钥矩阵. $P=K^{-1}\cdot C\pmod{26}$. Hill 一次替代多字母, 可以抵抗唯密文攻击, 但不抵抗已知明文攻击.

**Playfair 密码**: 多字母替换密码.