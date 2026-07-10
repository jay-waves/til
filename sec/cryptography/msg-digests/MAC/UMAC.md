## UMAC 

基于全域哈希 (Universal Hashing) 的 MAC 算法. 对于每次 MAC 计算, 采用不同的哈希算法, 这样对同一消息的多次 MAC 结果并不相同. 最常用的 UMAC 算法为 Poly1305, 描述于 [RFC8439](https://datatracker.ietf.org/doc/html/rfc8439).

## Poly1305

接收任意长度的消息 $M$, `256b` 长度的密钥 $K$, 输出 `128b` 的标签. 要求每次输入的密钥都不同, 因此属于全域哈希. 算法应**统一使用小端序**. 最初由 Daniel J. Bernstein 提出, 计算高效.

### 原理

Poly1305 基于有限域 $GF(2^{130} - 5)$, 其中 ${} P=2^{130}-5$ 是一个素数, 有限域是一个素域 $\mathbb{Z}_{P}$. 有限域的多项式映射有很好地**抗碰撞性**, 生成相同认证值的概率极低.

1. 消息分组: 消息 $M$ 被分成多个 16 字节的 $n$ 个分组 $m_i$, 每个分组被视为一个 128 位的整数 $$m_i = \text{bytes}[i \cdot 16 : (i+1) \cdot 16]$$ 如果最后一组不足 16 字节, 会在末位填充 `1`, 然后更高为补 `0`, 填充为 17 字节长度.
2. 生成多项式: 每个分组 $m_i$ 被看作多项式的系数, 以密钥 $r$ 为底数生成一个多项式, 即有限域上的一个值: $$P(M) = m_0 + m_1 \cdot r + m_2 \cdot r^2 + \dots + m_{n-1} \cdot r^{n-1}$$
3. 加盐: 多项式计算结果与另一密钥 $s$ 相加, 引入不可预测性. $$T = P(M) + s\pmod{2^{130}-5}$$

认证值最终为: $$\text{Tag} = \left( \left( \sum_{i=0}^{n-1} m_i \cdot r^i \right) + s \right) \mod (2^{130} - 5)$$

### 预处理

将数据转化为小端序; 将明文按 `128b` 长度分块, 最后一块不填充; 同时设置常数.

```python
p = (1 << 130) - 5
mask = ((1 << 128) - 1) # 128b

k: int = decode_by_little_endian( k: bytes ) # primary key

def chunked(data, chunk_size=16):
	for i in range(0, len(data), chunk_size):
		yield data[i:i + chunk_size]

for chunk in chunked(data: bytes):
	m: int = decode_by_little_endian(chunk)
	...
```

### 处理密钥

输入 `256b` 密钥 $K$, 分为左右两个 `128b` 串 $r, s$. 其中 $r$ 用于生成多项式, $s$ 用于 MAC 计算. 需要对比特串 $r$ 进行掩码操作 (clamping), 避免过大 $r$ 导致碰撞. 使 $r$ 在小端序下, 从右往左数:
- 4, 8, 12, 16 个字节的高 4 位为 `0`
- 5, 9, 13 个字节的最低 2 位为 `0`

```python
def clamp_r( r ):
	return r & 0x0ffffffc_0ffffffc_0ffffffc_0fffffff

r: int = (k >> 128) & mask
s: int = k & mask
clamp_r(r)
```

### 更新累加器

对于每个明文块:
1. 按小端序解释为二进制**整数** `m`
2. 设明文块的字节长度 `len`, 在 `m` 上加上 `(1 << len)`
3. 此时 `m` 的二进制字节长度应有 17, 否则应填充 `0` 比特. 对于数字 `m`, 此步骤可忽略.
7. 将累加器加上 `m`
8. 将累加器乘 `m` 后再模 `p`

```python
acc = 0 # accumulator
for chunk in chunked(data: bytes):
	m: int = decode_by_little_endian(chunk)
	m |= 1 << (len(chunk) * 8)
	if len(chunk) < 17: # size of last chunk need pad
		m = 
	acc += m
	acc = (r * acc) % p
acc += s
acc &= mask # ouput low 128b
result = encode_by_little_endian(acc)
```


> Poly1305 常与 ChaCha20 现代流密码结合使用. 