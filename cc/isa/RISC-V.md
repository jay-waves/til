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

没有标准状态寄存器. 

RISC-V 没有寄存器上下文管理指令 (如 [SPARC](SPARCv8.md) save/store), 必须手动操作寄存器管理上下文 (中断, 函数调用等).

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
- `SLTI rd, rs1, imm`: Set if Less Tahn Immediate, Signed
- `AUIPC rd, imm` : Add Upper Immediate to PC. $rd=PC+(imm20 \ll 12)$
- `LUI rd, imm` : Load Upper Immediate. `imm` 只有高 20b 有效, 低 12b 为零. 低 12b 一般用 `ADDI` 修改, 但需要注意其中 `imm` 的符号扩展行为. 

Alias:
- `NEG rd, rs`  : 实际是 `SUB rd, x0, rs`
- `NOP` : 实际是 `ADDI x0, x0, 0`
- `MV rd, rs` : 实际是 `ADDI rd, rs, 0`
- `NOT rd, rs` : 实际是 `XORI rd, rs, -1`
- `SGTZ rd, rs` : (set if greater than zero) `SLT rd, x0, rs`
- `SNEZ rd, rs` : (set if not equal to zero) `SLTU rd, x0, rs`
- `SEQZ rd, rs` : (set if equal to zero): `SLTIU rd, rs, 1`
- `LI rd, imm` 直接装载一个 32b 常量, 实际由 `LUI rd, imm[31:12]`, `ADDI rd, x0, imm[11:0]` 构成.
- `LA rd, symbol` Load Address. 实际由 `AUIPC rd, imm[31:12]` `ADDI rd, x0, imm[11:0]`. 


### 内存指令

一般指令只能操作寄存器, 只有 `load/store` 才能操作内存. 

$addr = rs_{1}+sign\_ext(imm12)$

- `LB rd, rs1, imm12`: 将字节装载到 `rd` 的最低字节, 然后将其符号扩展为 32b. 
- `LH rd, rs, imm`: load halfword
- `LW rd, rs, imm`: load word
- `LBU ...`: load byte, unsigned. 将字节装载到 `rd` 最低字节, 高位全部填充为0.
- `LHU ...`
- `SB rs2, rs1, imm`: 将 `rs2` 寄存器的最低字节放入内存.
- `SH rs2, rs1, imm`: 
- `SW ...`

内存屏障:
- `FENCE fm, pred, succ` 
- `FENCE.I` Instruction Fence

### 分支指令

分支指令主要采用 PC 相对寻址. 长跳转使用 JAL (20b).

#### 无条件分支

`JAL rd, imm20`: Jump and Link. 将 offset 加到 PC, 同时将顺序下一条指令的 PC 写入 rd.  imm 位宽为 20b, 被符号扩展后, 再左移一位 (保证地址两字节对齐). 

`JALR rd, rs, imm12`: $PC=(rs+offset)\&~1$, $offset=sign\_ext(imm)$. imm 只有 12b, 并且最终 PC 强制将最后一位清零. 

Alias:
- `J offset`: `JAL x0, offset`, no return address.
- `JAL offset`: `JAL x1, offset`. 函数调用时, 惯例将返回地址放在 `x1` 寄存器
- `JR rs`: `JALR x0, rs, 0`
- `RET`: `JALR x0, x1, 0`
- `CALL rd, symbol`: 函数调用. `AUIPC r, imm[31:12]`, `JALR x1, r, imm[11:0]`, 惯例采用 `x6` 作为 `r`. 
- `TAIL rs, symbol`: 尾调用. `AUIPC r, imm[31:12]`, `JALR x0, r, imm[11:0]`, 惯例采用 `x6` 作为 `r`. 用 `x0` 存储返回地址, 即, 不进行实际压栈行为.

#### 条件分支

$offset = sign\_ext(imm12)\ll 1$

- `BEQ rs1, rs2, imm12`: Branch if Equal 
- `BNE rs1, rs2, imm12`
- `BLT ...`
- `BGE ...`: Branch if Greater Than or Equal
- `BLTU ...`: (unsigned)
- `BGEU ...` 

Alias:
- `BGT rs, rt, imm` == `BLT rt, rs, imm`
- `BLE rs, rt, imm` == `BGE rt, rs, offset`
- `BEQZ rs, imm` == `BEQ rs, x0, imm`
- ...

## 内存模型


### 栈管理

寄存器必须一个一个用指令放到栈中或取出.

## C

## B

## 