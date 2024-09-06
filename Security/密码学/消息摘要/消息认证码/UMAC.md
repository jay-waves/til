## UMAC 

基于全域哈希 (Universal Hashing) 的 MAC 算法. 对于每次 MAC 计算, 采用不同的哈希算法, 这样对同一消息的多次 MAC 结果并不相同. 最常用的 UMAC 算法为 Poly1305, 描述于 [RFC8439](https://datatracker.ietf.org/doc/html/rfc8439).

## Poly1305

接收任意长度的消息 $M$, `256b` 长度的密钥 $K$, 输出 `128b` 的标签. 要求每次输入的密钥都不同, 因此属于全域哈希. 算法应**统一使用小端序**.

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

输入 `256b` 密钥 $K$, 分为左右两个 `128b` 串 $r, s$, $r$ 用于生成多项式, $s$ 用于 MAC 计算. 需要对比特串 $r$ 进行掩码操作 (clamping), 使其在小端序下, 从右往左数:
- 4, 8, 12, 16 个字节的高 4 位为 `0`
- 5, 9, 13 个字节的最低 2 位为 `0`

```python
def clamp_r( r ):
	return r & 0x0ffffffc_0ffffffc_0ffffffc_0fffffff

r: int = (k >> 128) & mask
s: int = k & mask
clamp_r(r)
```

#### 更新累加器

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
