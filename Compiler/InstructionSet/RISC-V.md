RISC-V 支持 32b, 64b, 128b. 小端序, RISC. 采用模块化设计, 有几种基本指令集和可选的扩展指令集. 基本指令级 + 监督指令集 (S) 就足以支持简单的 Unix 系统.

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

标准扩展指令集: 

| 名称     |                        | 版本 |
| -------- | ---------------------- | ---- |
| M        | 整数乘除法             | 2.0  |
| A        | 原子指令               | 2.1  |
| F        | 单精度浮点数           | 2.2  |
| D        | 双精度浮点数           | 2.2  |
| Zicsr    | 控制与状态寄存器       | 2.0  |
| Zifencei | 指令抓取屏障           | 2.0  |
| Q        | 四精度浮点数           | 2.2  |
| L        | 十进制浮点数           | x    |
| C        | 压缩指令               | 2.0  |
| B        | 位操作                 | 1.0  |
| J        | 动态指令翻译           | x    |
| T        | 顺序存储器访问         | x    |
| P        | SIMD                   | x    |
| V        | 向量运算               | x    |
| Zk       | 标量加密               | 1.0  |
| H        | Hypervisor             | 1.0  |
| S        | Supervisor             | 1.12 |
| Zam      | 非对齐不可中断指令     | x    |
| Zfh      | 半精度浮点数           | 1.0  |
| Zfinx    | 整数寄存器单精度浮点数 | 1.0  |
| Zmmul    | 整数纯乘法             | 1.0      |

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
| x8     | s0/fp    | saved reg / frame pointer          | callee |
| x9     | s1       | saved reg                          | callee |
| x10-11 | a0-1     | function arguments / return values | caller |
| x12-17 | a2-7     | function arguments                 | caller |
| x18-27 | s2-11    | saved registers                    | callee |
| x28-31 | t3-6     | temporaries                        | caller        |

一般指令只能操作寄存器, 只有 `load/store` 才能操作内存. 

RISC-V 没有寄存器上下文管理指令 (如 [SPARC](SPARCv8.md) save/store), 必须手动操作寄存器管理上下文 (中断, 函数调用等).

### 分支

函数调用使用 `JAL` (Jump and Link) 指令, 寄存器必须一个一个用指令放到栈中或取出.

没有条件码寄存器, 而是直接比较两个寄存器.

## 算术运算

加法, 减法, 位移, 位操作, 比较分支. 声称其他都可以用软件模拟. 

遇到 overflow / underflow / subnormal / divide by zero 时, 不会抛出异常, 而是产生一个合理的默认数值. 