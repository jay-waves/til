## IDEA

IDEA (International Data Encryption Algorithm) 是一种对称密钥加密算法, 最初由 Xuejia Lai 和 James Massey 于1991年设计.

由于专利版权和安全问题, 现被 AES 取代.

- 8 轮迭代.
- 每组 64 位数据, 输入输出也是 64 位.
- 使用 128 位密钥, 每轮需要 6 个子密钥, 输出时额外需要 4 个子密钥, 共 52 个子密钥.

***

### 符号定义

- 按位异或 <span style="color: blue;"> $\oplus$ </span>
- 模 $2^{16}$ 整数加法 <span style="color: green;"> $\boxplus$ </span>: 对于16位二进制数 $m_1, m_2$, $m_1\boxplus m_2=(m_1+m_2)\bmod{2^{16}}$.
- 模 $2^{16}+1$ 整数乘法 <span style="color: red;">$\odot$</span>: 对于16为二进制数 $m_1, m_2$, $m_1\odot m_2=(m_1\cdot m_2)\bmod{(2^{16}+1)}$.

$2^{16}+1=65537$ 是常见素数, $\odot$ 实际构成群.

### 轮结构

![图源wiki|400](../../../../attach/Pasted%20image%2020240326084540.png)

在每一轮中, 需要输入6个16位的子密钥, 轮结构中间结果为4组16位二进制串.

### 加密

8轮迭代后, 输出需要额外四个子密钥:

![图源wiki](../../../../attach/Pasted%20image%2020240326084510.png)

子密钥生成算法比较直接: 前8个子密钥 $Z_1, Z_2, \ldots, Z_8$ 直接在加密密钥中依次选取. 然后, 将加密密钥循环左移 $52$ 位, 再依次取接下来的8个子密钥. 以此类推. 