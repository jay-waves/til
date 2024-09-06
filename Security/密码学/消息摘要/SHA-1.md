> 详见 [SHA1与SM3算法文档](../../paper/crypto/SHA1%20and%20SM3%20implement.pdf)

SHA (Security Hash Algorithm) 由美国标准与技术研究所 (NIST) 设计, 并于 1993 年作为 FIPS180 标准发布, 该版本称为 SHA-0. 修订版 FIPS 180 于1995年发布, 称之为 SHA-1.

SHA-1 对输入最大长度有限制, 分组长度为 `512b`, 输出 `160b`

```python
from typing import List
import sys

# define some type
Int32 = int # 32bit int
Word = Int32
Int512 = int
BitLen = int
ByteLen = int

MASK = 0xffffffff

def _i2os(num, target_len: BitLen):
	'''数字转二进制字符串, sha1 使用大端序'''
	return num.to_bytes(target_len//8, 'big')

def _os2i(bstr):
	return int.from_bytes(bstr, 'big')

def _rotate(x: Int32, base: int) -> Int32:
	base = base % 32
	return ((x << base) & MASK) | (x >> (32-base))

def _cat_words(words: List[Word]):
	tmp = 0
	for word in words:
		tmp <<= 32
		tmp += word
	return tmp

def _fk(b: Word, c: Word, d: Word, t: int):
	if t >= 0 and t <= 19:
		f = b & c | (b ^ 0xffffffff) & d  # 与1异或实现取反
		k = 0x5a827999
	elif t >= 20 and t <= 39:
		f = b ^ c ^ d
		k = 0x6ed9eba1
	elif t >= 40 and t <= 59:
		f = b & c | b & d | c & d
		k = 0x8f1bbcdc
	elif t >= 60 and t <= 79:
		f = b ^ c ^ d
		k = 0xca62c1d6
	else:
		raise ValueError('round t is out of range')
	return (f+k) & MASK

def _padding(msg: bytes):
	'''padding msg end to 512*k length'''
	
	m_len: ByteLen = len(msg)
	pdd_len: ByteLen = (56-m_len%64)%64
	pdd1 = b'\x80' + b'\x00' * pdd_len 
	pdd2 = _i2os(m_len*8, 64)
	return msg + pdd1 + pdd2

def _extend(block: Int512):
	'''split 16 words, and transform into 80 words'''
	w: List[Word] = [0] * 80
	# divide vector into words
	for j in range(16):
		w[15-j] = block & MASK
		block >>= 32
	for j in range(16, 80):
		w[j] = _rotate(w[j-3] ^ w[j-8] ^ w[j-14] ^ w[j-16], 1)
	return w

def _divide(msg: bytes):
	'''divide msg into chunks'''
	assert len(msg)%64 == 0
	chunks = []
	chunk_size = 64
	for i in range(0, len(msg), chunk_size):
		chunks.append(msg[i:i+chunk_size])
	return chunks


# init global register with IV
h0,h1,h2,h3,h4 = (0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476, 0xc3d2e1f0)

# for each 512b chunk
def sha1_block(w: List[Word]):
	global h0, h1, h2, h3, h4
	a,b,c,d,e = h0,h1,h2,h3,h4
	for i in range(80):    
		 # 注意将f(b, c, d)+k 合并了
		temp = a
		a = (_rotate(a,5) + _fk(b,c,d,i) + e + w) & mask
		e = d
		d = c
		c = _rotate(b, 30)
		b = temp
	h0 = (h0+a) & mask
	h1 = (h1+b) & mask
	h2 = (h2+c) & mask
	h3 = (h3+d) & mask
	h4 = (h4+e) & mask

def sha1(msg: bytes) -> bytes:
	# 消息填充, 类型转化
	msg = padding(msg)
	blocks = msg_to_blocks(msg)

	for block in blocks:
		w = _extend(block)
		sha1_block(w)

	digest = _cat_words(reg)
	return _i2os(digest, 160)
```