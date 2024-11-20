
## SM4原理

基于 Feistel 结构，本质就是用*乱源*异或*每组明文*. 轮密钥由变换 T 生成, 输入是*其他组明文*与*本组轮密钥*, T 内部有线性变换和非线性变换, 线性变换用来扩散, 非线性变换用来混淆.

对于 SM4, 每组明文的形式是 $(X_{0}, X_{1}, X_{2}, X_{3})$, 由该组明文向后生成 $X_{4}$, 然后 $(X_{1}, X_{2}, X_{3}, X_{4})$ 变为新的明文组.
$X_{i+4}\ =\ X_{i}\oplus T(X_{i+1}\oplus X_{i+2}\oplus X_{i+3}\oplus rk_{i})$. 其中 $X_{i}$ 为一字, 等于四字节.

T 变换由线性变换 $L$ 和非线性变换 $\tau$ 组成: $T(m)\ =\ L(\tau(m))$, $\tau$ 涉及四个 S 盒变换, 每个盒输入一字节, 输出一字节; $L$ 则是循环移位和异或.

为了使加解密流程相同 ( Feistel 结构的特点), SM4 将最后密文组 $(X_{28}, X_{29}, X_{30}, X_{31})$ 取反作为输出, 类似 DES 的 $P^{-1}$ 置换. 此后 SM4 加解密的唯一区别就是轮密钥使用顺序相反. 

加密时, $X_{i+4}\ =\ X_{i}\oplus T(M)$  
解密时, $X_{i}\ =\ X_{i+4}\oplus T(M)\ =\ X_{i}\oplus T(M)\oplus T(M)\ =\ X_{i}$


## 源码

`sm4.py`

```python
"""
words: array of 4 bytes, like [B1, B2, B3, B4]
bytes: array of bytes
bytes: 8bits binary number
"""

import key_expansion as key
from constant import SBOX

def _non_linear_map(word):
    return bytes(SBOX.get(byte) for byte in word)

def _linear_map(word):
    """
    线性变换 L
    L(B) = B ^ (B <<< 2) ^ (B <<< 10) ^ (B <<< 18) ^ (B <<< 24)
    :param block: bytes, 32b
    """
    _loop_left_shift = lambda num, base: ((num << base) & 0xFFFFFFFF) | (num >> (32 - base))
	SCHEDULE = (2, 10, 18, 24)
	
    a = _w2i(word)
    b = a
    for shift in SCHEDULE:
	    b ^=i _loop_left_shift(a, shift)
	return _i2w(b)

def _round_f(words, rk):
    """
    轮函数变换
    F(X0, X1, X2, X3, rk) = X0 ^ T(X1 ^ X2 ^ X3 ^ rk)
    :param blocks: bytes (X0, X1, X2, X3), 4*32b
    :param rk: round keys, bytes, 32b
    """
    x0, x1, x2, x3 = words
    # x1 ^ x2 ^ x3 ^ rk
    a = _xor(x1, x2)
    b = _xor(a, x3)
    c = _xor(b, rk)
    
    d = _non_linear_map(c)
    e = _linear_map(d)
    x4 = _xor(x0, e)
    return x4

def _crypt(block, rk):
    """
    :param block: 16bytes
    :param rks: 16bytes
    """
	for i in range(32):
		blocks += _round_f(x[i:i+4], rks[i])

	# 反序变换 R
    return b''.join(blocks[-4:][::-1])

def encrypt(data, mk):
    """
    :param data: 16bytes
    :param mk: 16bytes
    """
    if len(data) != 16 or len(mk) != 16:
	    raise ValueError
	# 128b to 4*32b, bytes to words
	words = [data[i:i + 4] for i in range(0, 16, 4)]
    rks = key.rks_gen(mk) 
    return _crypt(words, rks)

def decrypt(data, mk):
    """
    :param data: 16bytes
    :param mk: 16bytes
    """
    if len(data) != 16 or len(mk) != 16:
	    raise ValueError
	# 128b to 4*32b, bytes to words
	words = [data[i:i + 4] for i in range(0, 16, 4)]
    rks = key.rks_gen(mk)[::-1]  # 逆序轮密钥
    return _crypt(words, rks)
	
```

`key_expansion.py`

```python
'''RK generation for SM4'''
from constant import SBOX, FK, CK
from utils import _xor, _w2i, _i2w

def _non_linear_map(word):
	return bytes( SBOX.get(byte) for byte in word )

def _linear_map(word):
    """
    线性变换 L'
    L'(B) = B ^ (B <<< 13) ^ (B <<< 23)
    :return: 1 word
    """
    _loop_left_shift = lambda num, base: ((num << base) & 0xFFFFFFFF) | (num >> (32 - base))
    SCHEDULE = (13, 23)

	a = _w2i(word)
	b = a
    for shift in SCHEDULE:
        b ^=i _loop_left_shift(a, shift)
    return _i2w(b)

def _round_f(word, ck):
    """
    轮函数 (线性 + 非线性变换)
    F(K) = K0 ^ T'(K1 ^ K2 ^ K3 ^ CK)
    :return: 1 word 
    """
    k0, k1, k2, k3 = word

    k4 = _xor(k1, k2)
    k4 = _xor(k4, k3)
    k4 = _xor(k4, ck)
    
    k4 = _non_linear_map(k4)
    k4 = _linear_map(k4)

    return _xor(k0, k4)

def rks_gen(mk):
    """
    :param mk: master key, 16bytes
    :return: list of 32 words
    """
    # K0,K1,K2,K3 = MK ^ FK
    k0 = _xor(mk[0 :4 ], FK[0])
    k1 = _xor(mk[4 :8 ], FK[1])
    k2 = _xor(mk[8 :12], FK[2])
    k3 = _xor(mk[12:16], FK[3])
    rks = [k0, k1, k2, k3]

    # 32 轮密钥生成
    for i in range(32):
        ks += _round_f(rks[i:i + 4], CK[i])

    return keys[4:]
```

`utils.py`

```python
def _xor(a, b):
    """
    对两个字节序列逐字节异或
    """
    return bytes(x ^ y for x, y in zip(a, b))

def _w2i(data):
    """
    将字节序列转化为整数
    """
    return int.from_bytes(data, 'big')

def _i2w(value, length=4):
    """
    将整数转化为 4 字节长的字节序列 (一字)
    """
    return value.to_bytes(length, 'big')
```

`constant.py`

```python
'''constant for SM4'''

# 系统参数 FK
FK = (0XA3B1BAC6, 0X56AA3350, 0X677D9197, 0XB27022DC)

# 固定参数 CK
CK = (0X00070E15, 0X1C232A31, 0X383F464D, 0X545B6269,
      0X70777E85, 0X8C939AA1, 0XA8AFB6BD, 0XC4CBD2D9,
      0XE0E7EEF5, 0XFC030A11, 0X181F262D, 0X343B4249,
      0X50575E65, 0X6C737A81, 0X888F969D, 0XA4ABB2B9,
      0XC0C7CED5, 0XDCE3EAF1, 0XF8FF060D, 0X141B2229,
      0X30373E45, 0X4C535A61, 0X686F767D, 0X848B9299,
      0XA0A7AEB5, 0XBCC3CAD1, 0XD8DFE6ED, 0XF4FB0209,
      0X10171E25, 0X2C333A41, 0X484F565D, 0X646B7279)

# S盒
SBOX = {
    0X00: 0XD6, 0X01: 0X90, 0X02: 0XE9, 0X03: 0XFE, 0X04: 0XCC, 0X05: 0XE1, 0X06: 0X3D, 0X07: 0XB7,
    0X08: 0X16, 0X09: 0XB6, 0X0A: 0X14, 0X0B: 0XC2, 0X0C: 0X28, 0X0D: 0XFB, 0X0E: 0X2C, 0X0F: 0X05,
    0X10: 0X2B, 0X11: 0X67, 0X12: 0X9A, 0X13: 0X76, 0X14: 0X2A, 0X15: 0XBE, 0X16: 0X04, 0X17: 0XC3,
    0X18: 0XAA, 0X19: 0X44, 0X1A: 0X13, 0X1B: 0X26, 0X1C: 0X49, 0X1D: 0X86, 0X1E: 0X06, 0X1F: 0X99,
    0X20: 0X9C, 0X21: 0X42, 0X22: 0X50, 0X23: 0XF4, 0X24: 0X91, 0X25: 0XEF, 0X26: 0X98, 0X27: 0X7A,
    0X28: 0X33, 0X29: 0X54, 0X2A: 0X0B, 0X2B: 0X43, 0X2C: 0XED, 0X2D: 0XCF, 0X2E: 0XAC, 0X2F: 0X62,
    0X30: 0XE4, 0X31: 0XB3, 0X32: 0X1C, 0X33: 0XA9, 0X34: 0XC9, 0X35: 0X08, 0X36: 0XE8, 0X37: 0X95,
    0X38: 0X80, 0X39: 0XDF, 0X3A: 0X94, 0X3B: 0XFA, 0X3C: 0X75, 0X3D: 0X8F, 0X3E: 0X3F, 0X3F: 0XA6,
    0X40: 0X47, 0X41: 0X07, 0X42: 0XA7, 0X43: 0XFC, 0X44: 0XF3, 0X45: 0X73, 0X46: 0X17, 0X47: 0XBA,
    0X48: 0X83, 0X49: 0X59, 0X4A: 0X3C, 0X4B: 0X19, 0X4C: 0XE6, 0X4D: 0X85, 0X4E: 0X4F, 0X4F: 0XA8,
    0X50: 0X68, 0X51: 0X6B, 0X52: 0X81, 0X53: 0XB2, 0X54: 0X71, 0X55: 0X64, 0X56: 0XDA, 0X57: 0X8B,
    0X58: 0XF8, 0X59: 0XEB, 0X5A: 0X0F, 0X5B: 0X4B, 0X5C: 0X70, 0X5D: 0X56, 0X5E: 0X9D, 0X5F: 0X35,
    0X60: 0X1E, 0X61: 0X24, 0X62: 0X0E, 0X63: 0X5E, 0X64: 0X63, 0X65: 0X58, 0X66: 0XD1, 0X67: 0XA2,
    0X68: 0X25, 0X69: 0X22, 0X6A: 0X7C, 0X6B: 0X3B, 0X6C: 0X01, 0X6D: 0X21, 0X6E: 0X78, 0X6F: 0X87,
    0X70: 0XD4, 0X71: 0X00, 0X72: 0X46, 0X73: 0X57, 0X74: 0X9F, 0X75: 0XD3, 0X76: 0X27, 0X77: 0X52,
    0X78: 0X4C, 0X79: 0X36, 0X7A: 0X02, 0X7B: 0XE7, 0X7C: 0XA0, 0X7D: 0XC4, 0X7E: 0XC8, 0X7F: 0X9E,
    0X80: 0XEA, 0X81: 0XBF, 0X82: 0X8A, 0X83: 0XD2, 0X84: 0X40, 0X85: 0XC7, 0X86: 0X38, 0X87: 0XB5,
    0X88: 0XA3, 0X89: 0XF7, 0X8A: 0XF2, 0X8B: 0XCE, 0X8C: 0XF9, 0X8D: 0X61, 0X8E: 0X15, 0X8F: 0XA1,
    0X90: 0XE0, 0X91: 0XAE, 0X92: 0X5D, 0X93: 0XA4, 0X94: 0X9B, 0X95: 0X34, 0X96: 0X1A, 0X97: 0X55,
    0X98: 0XAD, 0X99: 0X93, 0X9A: 0X32, 0X9B: 0X30, 0X9C: 0XF5, 0X9D: 0X8C, 0X9E: 0XB1, 0X9F: 0XE3,
    0XA0: 0X1D, 0XA1: 0XF6, 0XA2: 0XE2, 0XA3: 0X2E, 0XA4: 0X82, 0XA5: 0X66, 0XA6: 0XCA, 0XA7: 0X60,
    0XA8: 0XC0, 0XA9: 0X29, 0XAA: 0X23, 0XAB: 0XAB, 0XAC: 0X0D, 0XAD: 0X53, 0XAE: 0X4E, 0XAF: 0X6F,
    0XB0: 0XD5, 0XB1: 0XDB, 0XB2: 0X37, 0XB3: 0X45, 0XB4: 0XDE, 0XB5: 0XFD, 0XB6: 0X8E, 0XB7: 0X2F,
    0XB8: 0X03, 0XB9: 0XFF, 0XBA: 0X6A, 0XBB: 0X72, 0XBC: 0X6D, 0XBD: 0X6C, 0XBE: 0X5B, 0XBF: 0X51,
    0XC0: 0X8D, 0XC1: 0X1B, 0XC2: 0XAF, 0XC3: 0X92, 0XC4: 0XBB, 0XC5: 0XDD, 0XC6: 0XBC, 0XC7: 0X7F,
    0XC8: 0X11, 0XC9: 0XD9, 0XCA: 0X5C, 0XCB: 0X41, 0XCC: 0X1F, 0XCD: 0X10, 0XCE: 0X5A, 0XCF: 0XD8,
    0XD0: 0X0A, 0XD1: 0XC1, 0XD2: 0X31, 0XD3: 0X88, 0XD4: 0XA5, 0XD5: 0XCD, 0XD6: 0X7B, 0XD7: 0XBD,
    0XD8: 0X2D, 0XD9: 0X74, 0XDA: 0XD0, 0XDB: 0X12, 0XDC: 0XB8, 0XDD: 0XE5, 0XDE: 0XB4, 0XDF: 0XB0,
    0XE0: 0X89, 0XE1: 0X69, 0XE2: 0X97, 0XE3: 0X4A, 0XE4: 0X0C, 0XE5: 0X96, 0XE6: 0X77, 0XE7: 0X7E,
    0XE8: 0X65, 0XE9: 0XB9, 0XEA: 0XF1, 0XEB: 0X09, 0XEC: 0XC5, 0XED: 0X6E, 0XEE: 0XC6, 0XEF: 0X84,
    0XF0: 0X18, 0XF1: 0XF0, 0XF2: 0X7D, 0XF3: 0XEC, 0XF4: 0X3A, 0XF5: 0XDC, 0XF6: 0X4D, 0XF7: 0X20,
    0XF8: 0X79, 0XF9: 0XEE, 0XFA: 0X5F, 0XFB: 0X3E, 0XFC: 0XD7, 0XFD: 0XCB, 0XFE: 0X39, 0XFF: 0X48
}
```