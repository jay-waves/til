我这拿整型实现真是贱呐，给自己没事找事

- P: permutate
- C: compress
- S: substitute
- LK: left_key_28b
- RK: right_key_28b
- rK: round_key_48b

```python
import table
from table import permutate
import key

def F_Sbox(data_48b):
    '''
    Sbox代换
    input: 扩展并和Ki异或后的数据 data_48b
    return: 经S盒代换后的数据 data_32b
    '''
    data_32b = 0
    for i in range(8):
        data = (data_48b >> (6*(7-i))) & 0x3F
        row = (data & 0x20) >> 4 | (data & 0x01) #取出首尾位
        col = (data & 0x1E) >> 1 #取出中间四位
        data_32b = (data_32b << 4) | table.Sbox[i][row][col]
    return data_32b

def F_IP(data_64b):
    '''
    IP置换
    input: 初始数据 64b
    return: 经IP置换后的数据 64b
    '''
    return permutate(table.IP, data_64b, 64)

def F_IP_inv(data_64b):
    '''
    IP置换
    input: 轮加密后数据 64b
    return: 经IP逆置换后的数据 64b
    '''
    return permutate(table.IP_inv, data_64b, 64)

def F_E(R_32b):
    '''
    E扩展置换
    input: 上轮加密后R 32b
    return: 经E扩展置换后的数据 48b
    '''
    return permutate(table.extension, R_32b, 32)
    
def F_P(data_32b):
    '''
    Pbox置换
    input: S盒代换压缩后的数据 32b
    return: 经Pbox置换后的数据 32b
    '''
    return permutate(table.Pbox, data_32b, 32)

def F_round(cnt, rk, L_32b, R_32b):
    '''
    Feistel轮加密主体
    input: 
        cnt: 轮数 
        rk: 轮密钥 上
        L_32b: 上一轮左半部分 32b
        R_32b: 上一轮右半部分 32b           
    return: 
        下一轮 左半部分32b, 右半部分32b
    '''
    data_48b = F_E(R_32b)
    data_48b = rk ^ data_48b
    data_32b = F_Sbox(data_48b)
    data_32b = F_P(data_32b)
    data_32b = data_32b ^ L_32b
    return R_32b, data_32b

def DES(k, data, is_enc=True)->int:
    '''
    input:
        k: 初始密钥
        data: 待加密/解密数据
        op: 模式, 1代表加密, 0代表解密
    return:
        (int)加密/解密后的数据
    '''
    #16轮密钥生成
    K = key.rk_gen(k)

    #初始化数据
    data0 = F_IP(data)
    L = data0 >>32
    R = data0 & 0xFFFF_FFFF
    
	for cnt in range(16):
		i = cnt if is_enc else (15-cnt)
		L, R = F_round(cnt, K[i], L, R)
    
    data_64b = (R<<32) | L
    cipher = F_IP_inv(data_64b)
    return cipher
```

密钥扩展模块 key.py
```python
import table
from table import permutate

def PC1(key_64b):
    '''
    PC1置换
    input: 初始密钥 key_64b 
    return: 置换压缩后-初始轮密钥 rkey0_56b
    '''
    return permutate(table.key_PC_1, key_64b, 64)

def PC2(LK_28b, RK_28b):
    '''
    PC2置换
    input: 左移位后的左右轮密钥 2*28b
    return: 本轮Feistel子密钥 rKi_48b
    '''
    return permutate(table.key_PC_2, (LK_28b<<28) + RK_28b, 56)

def round(LK_28b, RK_28b, cnt):
    '''
    单轮子密钥生成
    input: 上一轮左右轮密钥 2*28b, 当前轮数 cnt
    return: 新一轮左右轮密钥 2*28b, 本轮Feistel子密钥 rKi_48b
    '''
    left_shift_bin = table.key_left_shift_schedule[cnt]
    LK_28b = ((LK_28b << left_shift_bin) & 0xFFFFFFF) | (LK_28b >> (28-left_shift_bin))
    RK_28b = ((RK_28b << left_shift_bin) & 0xFFFFFFF) | (RK_28b >> (28-left_shift_bin))
    # 采用int类型存储数据，不能使用截取
    return LK_28b, RK_28b, PC2(LK_28b, RK_28b)

def rk_gen(K):
    '''
    16轮密钥生成
    return 16轮密钥数组
    '''
    K = PC1(K)
    LK = K >> 28
    RK = K & 0xFFF_FFFF
    rks = [0 for _ in range(16)]
    for i in range(16):
        LK, RK, rks[i] = round(LK, RK, i)
    return rks
```

常数以及替换盒模块 table.py
```python
def permutate(table, data, data_size):
    '''
    Input: 
        table: permutate ref teble
        data: init data (int)
    Return:
        permutated data
    '''
    result = 0
    size = len(table)
    for i in range(size):
        # 数据是小端处理，数据高位在地址低位（从左）
        result |= ( (data >> (data_size-table[i])) & 0x1) << (size - i-1)
    return result

IP = [
    58, 50, 42, 34, 26, 18, 10,  2, 60, 52, 44, 36, 28, 20, 12,  4,
    62, 54, 46, 38, 30, 22, 14,  6, 64, 56, 48, 40, 32, 24, 16,  8,
    57, 49, 41, 33, 25, 17,  9,  1, 59, 51, 43, 35, 27, 19, 11,  3,
    61, 53, 45, 37, 29, 21, 13,  5, 63, 55, 47, 39, 31, 23, 15,  7] # 4*16=64
IP_inv = [
    40,  8, 48, 16, 56, 24, 64, 32, 39,  7, 47, 15, 55, 23, 63, 31,
    38,  6, 46, 14, 54, 22, 62, 30, 37,  5, 45, 13, 53, 21, 61, 29,
    36,  4, 44, 12, 52, 20, 60, 28, 35,  3, 43, 11, 51, 19, 59, 27,
    34,  2, 42, 10, 50, 18, 58, 26, 33,  1, 41,  9, 49, 17, 57, 25]
key_PC_1 = [
    57, 49, 41, 33, 25, 17,  9,  1, 58, 50, 42, 34, 26, 18,
    10,  2, 59, 51, 43, 35, 27, 19, 11,  3, 60, 52, 44, 36,
    63, 55, 47, 39, 31, 23, 15,  7, 62, 54, 46, 38, 30, 22,
    14,  6, 61, 53, 45, 37, 29, 21, 13,  5, 28, 20, 12,  4] # 4*14 = 56
key_PC_2=[
    14, 17, 11, 24,  1,  5,  3, 28, 15,  6, 21, 10,
    23, 19, 12,  4, 26,  8, 16,  7, 27, 20, 13,  2,
    41, 52, 31, 37, 47, 55, 30, 40, 51, 45, 33, 48,
    44, 49, 39, 56, 34, 53, 46, 42, 50, 36, 29, 32] # 4*12=48
extension=[
    32,  1,  2,  3,  4,  5,  4,  5,  6,  7,  8,  9,
    8,  9, 10, 11, 12, 13, 12, 13, 14, 15, 16, 17,
    16, 17, 18, 19, 20, 21, 20, 21, 22, 23, 24, 25,
    24, 25, 26, 27, 28, 29, 28, 29, 30, 31, 32,  1 ] #48
key_left_shift_schedule = [
     1, 1, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 1] #17, 
Pbox = [
    16,  7, 20, 21, 29, 12, 28, 17,  1, 15, 23, 26,  5, 18, 31, 10,
    2,  8, 24, 14, 32, 27,  3,  9, 19, 13, 30,  6, 22, 11,  4, 25]
Sbox=[[
    [14, 4, 13, 1, 2, 15, 11, 8, 3, 10, 6, 12, 5, 9, 0, 7],
    [0, 15, 7, 4, 14, 2, 13, 1, 10, 6, 12, 11, 9, 5, 3, 8],
    [4, 1, 14, 8, 13, 6, 2, 11, 15, 12, 9, 7, 3, 10, 5, 0],
    [15, 12, 8, 2, 4, 9, 1, 7, 5, 11, 3, 14, 10, 0, 6, 13]]
,[
    [15, 1, 8, 14, 6, 11, 3, 4, 9, 7, 2, 13, 12, 0, 5, 10],
    [3, 13, 4, 7, 15, 2, 8, 14, 12, 0, 1, 10, 6, 9, 11, 5],
    [0, 14, 7, 11, 10, 4, 13, 1, 5, 8, 12, 6, 9, 3, 2, 15],
    [13, 8, 10, 1, 3, 15, 4, 2, 11, 6, 7, 12, 0, 5, 14, 9]]
,[
    [10, 0, 9, 14, 6, 3, 15, 5, 1, 13, 12, 7, 11, 4, 2, 8],
    [13, 7, 0, 9, 3, 4, 6, 10, 2, 8, 5, 14, 12, 11, 15, 1],
    [13, 6, 4, 9, 8, 15, 3, 0, 11, 1, 2, 12, 5, 10, 14, 7],
    [1, 10, 13, 0, 6, 9, 8, 7, 4, 15, 14, 3, 11, 5, 2, 12]]
,[
    [7, 13, 14, 3, 0, 6, 9, 10, 1, 2, 8, 5, 11, 12, 4, 15],
    [13, 8, 11, 5, 6, 15, 0, 3, 4, 7, 2, 12, 1, 10, 14, 9],
    [10, 6, 9, 0, 12, 11, 7, 13, 15, 1, 3, 14, 5, 2, 8, 4],
    [3, 15, 0, 6, 10, 1, 13, 8, 9, 4, 5, 11, 12, 7, 2, 14]]
,[
    [2, 12, 4, 1, 7, 10, 11, 6, 8, 5, 3, 15, 13, 0, 14, 9],
    [14, 11, 2, 12, 4, 7, 13, 1, 5, 0, 15, 10, 3, 9, 8, 6],
    [4, 2, 1, 11, 10, 13, 7, 8, 15, 9, 12, 5, 6, 3, 0, 14],
    [11, 8, 12, 7, 1, 14, 2, 13, 6, 15, 0, 9, 10, 4, 5, 3]]
,[
    [12, 1, 10, 15, 9, 2, 6, 8, 0, 13, 3, 4, 14, 7, 5, 11],
    [10, 15, 4, 2, 7, 12, 9, 5, 6, 1, 13, 14, 0, 11, 3, 8],
    [9, 14, 15, 5, 2, 8, 12, 3, 7, 0, 4, 10, 1, 13, 11, 6],
    [4, 3, 2, 12, 9, 5, 15, 10, 11, 14, 1, 7, 6, 0, 8, 13]]
,[
    [4, 11, 2, 14, 15, 0, 8, 13, 3, 12, 9, 7, 5, 10, 6, 1],
    [13, 0, 11, 7, 4, 9, 1, 10, 14, 3, 5, 12, 2, 15, 8, 6],
    [1, 4, 11, 13, 12, 3, 7, 14, 10, 15, 6, 8, 0, 5, 9, 2],
    [6, 11, 13, 8, 1, 4, 10, 7, 9, 5, 0, 15, 14, 2, 3, 12]]
,[
    [13, 2, 8, 4, 6, 15, 11, 1, 10, 9, 3, 14, 5, 0, 12, 7],
    [1, 15, 13, 8, 10, 3, 7, 4, 12, 5, 6, 11, 0, 14, 9, 2],
    [7, 11, 4, 1, 9, 12, 14, 2, 0, 6, 10, 13, 15, 3, 5, 8],
    [2, 1, 14, 7, 4, 10, 8, 13, 15, 12, 9, 0, 3, 5, 6, 11]]]
```
