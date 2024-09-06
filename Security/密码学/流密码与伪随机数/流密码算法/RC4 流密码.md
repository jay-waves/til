## RC4算法

RC4, Rivest Cipher 4, 是一种[**流密码**](../流密码.md)算法, 由Ron Rivest设计. 简单高效, 但近年来发现一些安全漏洞. 其在速度上比较AES高效软件实现已不占优.

RC4用一S表生成序列密钥, 主要步骤一是**密钥调度算法** (KSA), 使用初始密钥生成S表; 二是**伪随机数生成算法** (PRGA), 利用S表生成伪随机数序列.

### 1 密钥调度算法

KSA, Key Scheduling Algorithm, 使用初始密钥完成S表初始化. 

初始密钥长度应小于等于256字节, KSA算法将其扩展为调度表(S表), S表是256维向量, 分量长1字节. 实现过程如下:

0. 初始化两计数器 $i=0$, $j=0$
1. $S[i]=i,\ 0\leq i\lt 255$, 线性填充
2. $For\quad i\ = 0\ \to 255:$  
	$\quad j=j+S[i]+K[i\pmod{256}]\pmod{256}$, K为字节串初始密钥.  
	$\quad Swap(\ S[i],\ S[j]\ )$

### 2 伪随机数生成算法

PRGA, Pseudo-Random Generation Algorithm, 生成和明文等长**字节密钥流**.

1. 初始化计数器 $i=0$, $j=0$
2. 如下生成随机字节:  
	- $i=i+1\pmod{256}$  
	- $j=j+S[i]\pmod{256}$  
	- $Swap(\ S[i],\ S[j]\ )$  
	- $t=S[i]+S[j]\pmod{256}$  
	- $Ks=S[t]$, 输出字节.

### 代码实现

```python
import sys

class RC4:
    def __init__(self, key: bytes) -> None:
        """
        Initialize the RC4 cipher.
        """
        self.S = self.KSA(key) #key用完丢弃
        self.i = 0
        self.j = 0

    def KSA(self, key) -> list:
        """
        Key Scheduling Algorithm
        """
        key_length = len(key)
        S = list(range(256))
        j = 0
        for i in range(256):
            j = (j + S[i] + key[i % key_length]) % 256
            S[i], S[j] = S[j], S[i]
        return S

    def PRGA(self) -> int:
        """
        Pseudo-Random Generation Algorithm (PRGA)
        :return: Ks, 单字节整型
        """
        i = (self.i + 1) % 256
        j = (self.j + self.S[i]) % 256
        self.S[i], self.S[j] = self.S[j], self.S[i]
        
        t = (self.S[i] + self.S[j]) % 256
        Ks = self.S[t]
        
        self.i, self.j = i, j
        return Ks

    def encrypt_chunk(self, chunk):
        """Encrypt a chunk of data using the RC4 stream cipher.
        """
        ciphertext = bytearray(len(chunk))
        for idx, byte in enumerate(chunk):
            ciphertext[idx] = byte ^ self.PRGA()
        return ciphertext

    def decrypt_chunk(self, chunk):
        return self.encrypt_chunk(chunk)  # RC4加密和解密过程相同

def main():
    key = input().strip()[2:] # 去掉0x
    key = bytes.fromhex(key)

    rc4 = RC4(key)
    # 为了加速，可以提前缓存随机流，放这里

    chunk_size = 64
    buffer = ''
    sys.stdin.read(2) # 去掉0x

    print('0x', end='')
    while True:
        byte = sys.stdin.read(1)
        if not byte:
            break
        else:
            buffer += byte

        if len(buffer) >= chunk_size:
            chunk = bytes.fromhex(buffer)
            cipher = rc4.encrypt_chunk(chunk)  # 加密 chunk
            buffer = ''
            print(cipher.hex(), end='')

    chunk = bytes.fromhex(buffer.strip())
    cipher = rc4.encrypt_chunk(chunk)  # 加密 chunk
    print(cipher.hex())

if __name__ == '__main__':
    main()
```