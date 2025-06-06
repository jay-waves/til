### 寄存器

寄存器均为 32b, 建议使用小端序.
- `R0 - R12` 通用寄存器
- `SP (R13)` 堆栈寄存器
- `LR (R14)` 链接寄存器, 用于存储返回地址. 用于 `bl` (branch and link) 指令返回. 初始化为 `0xffffffff`
- `PC (R15)` 程序计数器. 

程序状态寄存器 (APSR, Application Program Status Register)
- N: negative 
- Z: zero 
- C: Carry. 
- V: Overflow 
- Q: 饱和加法标识
- GE: DSP only. 用于 SIMD 长数据加法.

```
31 | 30 | 29 | 28 | 27 |        20 | 19 18 17 16 | 15               0 |
N  | Z  | C  | V  | Q  | Reserved  |    GE       |       Reserved     |
```

### 内存模型


#### 内存映射



- Code 
- SRAM 
- Peripheral 
- RAM Regions * 2
- Device Regions * 2
- System: `0xe0000000 ~ 0xffffffff` is reserved for system-level use 


### 向量表

### 中断模型

