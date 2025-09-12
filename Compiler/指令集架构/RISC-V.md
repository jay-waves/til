RISC-V 支持 32b, 64b, 128b. 按字节寻址, 小端序, RISC. 采用模块化设计, 有几种基本指令集和可选的扩展指令集. 基本指令级 + 监督指令集 (S) 就足以支持简单的 Unix 系统.

## ISA 

基本指令集: (x 表示指令集仍未稳定)

| 名称   |                                       | 版本 |
| ------ | ------------------------------------- | ---- |
| RVWMO  | RISC-V 弱内存模型                     | 2.0  |
| RV32I  | 基本整数指令集, 32b                   | 2.1  |
| RV32E  | 基本整数指令集 (嵌入式), 32b, 16 regs | 2.0  |
| RV64I  | 基本整数指令集, 64b                   | 2.1  |
| RV64E  | 基本整数指令集 (嵌入式), 64b, 16 regs | 2.0  |
| RV128I | 基本整数指令集, 128b                  | x    |

RV64I 向下兼容 RV32I 指令编码, 但是行为有所不同: 32b 指令的结果存储在低 32b, 但是符号位却在高 32b, 官方文档承认这可能是个设计错误. RV32I 的指令字长有 4 字节, 也有 2 字节.

标准扩展指令集: 

| 名称     |                                                     | 版本 |
| -------- | --------------------------------------------------- | ---- |
| A        | 原子指令 (Atomic Insn)                              | 2.1  |
| B        | 位操作 (Bit Manipulation)                           | 1.0  |
| C        | 压缩指令 (Compressed Insn)                          | 2.0  |
| D        | 双精度浮点数 (Double-Precision FP)                  | 2.2  |
| F        | 单精度浮点数 (Single-Precision FP)                  | 2.2  |
| H        | Hypervisor                                          | 1.0  |
| I        | 基础指令集 (Base)                                   |      |
| J        | 动态指令翻译                                        | x    |
| K        | 标量加密 (Scalar Cryptography)                      | 1.0  |
| L        | 十进制浮点数 (Decimal FP)                           | x    |
| M        | 整数乘除法 (Integer Multi & Div)                    | 2.0  |
| N        | 用户侧中断 (User-Level Interrupts)                  |      |
| P        | Packed SIMD                                         | x    |
| Q        | 四精度浮点数 (Quad-Precision FP)                    | 2.2  |
| S        | Supervisor                                          | 1.12 |
| T        | 顺序存储器访问 (Transactional Memory)               | x    |
| V        | 向量运算 (Vector Operations)                        | x    |
| Zicsr    | 控制与状态寄存器 (Control and Status Register Insn) | 2.0  |
| Zifencei | 指令抓取屏障                                        | 2.0  |
| Zam      | 非对齐不可中断指令 (Misaligned Atomics)             | x    |
| Zfh      | 半精度浮点数                                        | 1.0  |
| Zfinx    | 整数寄存器单精度浮点数                              | 1.0  |
| Zmmul    | 整数纯乘法                                          | 1.0  |
| Ztso    | TSO                                          |   |

### 指令编码

RV32I:
- R-Type : R1, R2 --> R3. 
- I-Type : R1, Imm --> R2. Imm 占 12b, 被符号扩展为 32b.
- S-Type : Store Op.
- B-Type : Branch Op.
- U-Type : wider immediate data (20b most-significant bits)
- J-Type : Jump Op. 

## 寄存器 

标准版有 32 个整数寄存器, 嵌入式版本有 16 个 (`r0~r15`). 有浮点数扩展时, 还有 32 个浮点数寄存器. `r0` 为*零寄存器*, 赋值无用, 读取得零.

| Reg    | ABI Name |                                    | Saver  |
| ------ | -------- | ---------------------------------- | ------ |
| x0     | zero     | Hard-wired zero                    |        |
| x1     | ra       | return address                     | caller |
| x2     | sp       | stack pointer                      | callee |
| x3     | gp       | global pointer                     |        |
| x4     | tp       | thread pointer                     |        |
| x5-7   | t0-2     | temporaries                        | caller |
| x8     | s0/fp    | saved reg0 / frame pointer         | callee |
| x9     | s1       | saved reg1                         | callee |
| x10-11 | a0-1     | function arguments / return values | caller |
| x12-17 | a2-7     | function arguments                 | caller |
| x18-27 | s2-11    | saved registers                    | callee |
| x28-31 | t3-6     | temporaries                        | caller        |

一般指令只能操作寄存器, 只有 `load/store` 才能操作内存. 

RISC-V 没有寄存器上下文管理指令 (如 [SPARC](SPARCv8.md) save/store), 必须手动操作寄存器管理上下文 (中断, 函数调用等).

### 分支

函数调用使用 `JAL` (Jump and Link) 指令, 寄存器必须一个一个用指令放到栈中或取出.

没有条件码寄存器, 而是直接比较两个寄存器.

## RV32I

### 算术运算

加法 / 减法 / 位移 / 位操作 / 比较分支. 其他都可以用基础指令来模拟.

- `ADD rd, rs1, rs2` / `ADDI rd, rs1, imm`
- `SUB rd, rs1, rs2` :  rs1 - rs2
- `SLL rd, rs1, rs2`: : shift left logical. rs2 的最小 5b 决定移位值.
- XOR rd, rs1, rs2
- `SRL rd, rs1, rs2`: shift right logical. 填充零.
- `SRA rd, rs1, rs2`: shift right arithmetic. 填充符号位.
- `OR rd, rs1, rs2`. 注意, `ORI rd, rs1, imm` 中立即数会被符号扩展, 并非"按位或"行为.
- `AND rd, rs1, rs2`: 
- `SLT rd, rs1, rs2`: Set if Less Than. 如果 rs1 < rs2, 设置 `rd=1`, 否则 `rd=0`
- `SLTU rd, rs1, rs2`: Set if Less Than, Unsigned

Alias:
- `NEG rd, rs`  : 实际是 `SUB rd, x0, rs`
- `NOP` : 实际是 `ADDI x0, x0, 0`
- `MV rd, rs` : 实际是 `ADDI rd, rs, 0`
- `NOT rd, rs` : 实际是 `XORI rd, rs, -1`
- `SGTZ rd, rs` : (set if greater than zero) `SLT rd, x0, rs`
- `SNEZ rd, rs` : (set if not equal to zero) `SLTU rd, x0, rs`


### 内存指令

- LB rd, offset(rs1)
- LH rd, offset(rs1)
- LW rd, offset(rs1)
- LBU rd, offset(rs1)
- LHU rd, offset(rs1)
- SB rs2, offset(rs1)
- SH rs2, offset(rs1)
- SW rs2, offset(rs1)

### 分支指令

- BEQ rs1, rs2, offset 
- BNE 
- BLT
- BGE 
- BLTU 
- BGEU 

## C

## B

## 