|      | ARMv7-M             | ARMv8-M                |
| ---- | ------------------- | ---------------------- |
| date | 2005                | 2015                   |
| CPUs | Cortex-M3 / M4 / M7 | Cortex-M23 / M33 / M55 |
| ISA  | Thumb-2             | Thumb2                 |
|      |                     | TrustZone, MVE (向量扩展)              |

具体硬件型号见 [Cortex-M](../../HardWare/嵌入式/Cortex-M.md)

## 寄存器

寄存器均为 32b, 建议使用小端序.
- `R0 - R12` 通用寄存器
- `SP (R13)` 堆栈寄存器
- `LR (R14)` 链接寄存器, 用于存储返回地址. 用于 `bl` (branch and link) 指令返回. 初始化为 `0xffffffff`
- `PC (R15)` 程序计数器. 

程序状态寄存器 (APSR, Application Program Status Register)
- N: Negative 
- Z: Zero 
- C: Carry. 
- V: Overflow 
- Q: 饱和加法标识
- GE: DSP only. 用于 SIMD 长数据加法.

```
31 | 30 | 29 | 28 | 27 |        20 | 19 18 17 16 | 15               0 |
N  | Z  | C  | V  | Q  | Reserved  |    GE       |       Reserved     |
```

### SP

- Main stack Pointer: Handler mode or Thread Mode
- Process Stack Pointer: Thread Mode 

## 内存模型


### 内存映射


每个内存区域最多支持 0.5GB (受指令字长限制).
- Code 
- SRAM 
- Peripheral 
- RAM Regions * 2
- Device Regions * 2
- System: PPB (Private Peripheral Bus), 

| Name       | Address      | EndAddress   | Device Type           | Attribute | Description                              |
| ---------- | ------------ | ------------ | --------------------- | --------- | ---------------------------------------- |
| Code       | `0x00000000` | `0x1fffffff` | Normal                | WT        | ROM, or Flash                            |
| SRAM       | `0x20000000` | `0x3fffffff` | Normal                | WBWA      | SRAM region, used for on-chip RAM        |
| Peripheral | `0x40000000` | `0x5fffffff` | Device                | XN        | On-chip peripheral                       |
| RAM        | `0x60000000` | `0x7fffffff` | Normal                | WBWA      | Memory with WBWA for L2/L3 cache support |
| RAM        | `0x80000000` | `0x9fffffff` | Normal                | WT        | Memory with non-cached                   |
| Device     | `0xa0000000` | `0xbfffffff` | Device, shareable     | XN        |                                          |
| Device     | `0xc0000000` | `0xdfffffff` | Device, non-shareable | XN        |                                          |
| System     | `0xe0000000` | `0xfffffff`  | ...                   | XN        | PPB, or vendor system peripherals                                          |

ARM System-Level 是指操作系统相关功能的指令集支持, 包括 指令权限分级 / 系统调用 / 中断响应.  在嵌入式环境中, Application-Level 和 System-Level 的分野并不明显.

#### PPB 

PPB: `0xe0000000~0xe00fffff`. 其中 `0xe000e000~0xe000efff` 4KB 是 SCS (System Control Space)
- Processor ID registers 
- general control and configuration, including vector table base address 
- system handler support, for interrupts and exceptions 
- system timer, SysTick 
- Nested Vectored Interrupt Controller (NVIC): external interrupts 
- Fault status, control registers 
- Protected Mmeory System Architecture, PMSAv7 
- Cache and barnch predictor control 
- Processor debug 

SCS 控制着所有的 exceptions, interrupts, external interrupts 

### 内存权限

- XN: Execute Never. 不能执行的内存, 执行会触发 MemManage Exception 
- IMPLEMENTATION DEFINED: (标准不定义的权限)
	- Read-werite 
	- Read-only 
	- No-access. unpopulated parts of the address map.

缓存类型:
- WT : Write-through, non-cached 
- WBWA: Write-back, write allocate. write-through or  non-cached. 

内存序:
- Shareable: 允许多个端在 coherent memory domain 中共用该内存. 如多个处理器和 DMA 设备.
- SO: Strongly-ordered memory. SO 总意味着 Shareable.

### 内存同步

## 异常控制

Exception Categories:
- Reset: execution restarts from a fixed point 
- Supervisor call (SVCall): SVC insn. Application software uses SVC isntruction to make a call to an underlying operating system. Interrupt-Driven Supervisor-calling mechanism, PendSV 
- Fault: error condition in execution, reported synchronouosly or asynchronously 
- Interrupt: asynchronous to the instruction stream.  

Exception State:
- Inactive 
- Pending: exception generated, but processor not ye started processing. 
- Active: exception for which processor has started executing in a corresponding exception handler (ISR), but has not returned yet. handler is either running or preempted by a handler for a higher priority exception.
- Active and pending: one instance of the exception is active, and a second instance of the exception is pending.  (only asynchronous exceptions can be this state)

Mode:
- Thread Mode: entered on reset, and can be entered as a result of an exepction return
- Handler Mode: entered as a result of an exception. Execution in Handler Mode is always priviledged. 

### 向量表

中断表, 异常号是向量表中的字偏移量 (Word offset)

| Exception Number | Exception                    | Description                                                                        | Priority |
| ---------------- | ---------------------------- | ---------------------------------------------------------------------------------- | -------- |
| 0                | SP_main.                     | reset value of the main stack pointer                                                                                   |          |
| 1                | Reset                        | Power-on Reset / Local Reset (for debug mode)                                      | -3       |
| 2                | NMI (Non Markable Interrupt) | highest priority exception other than reset                                        | -2       |
| 3                | HardFault                    | generic hardware fault, for unrecoverable system failures                          | -1       |
| 4                | MemManage                    | memory protection faults for both data and instrution memory transactions          |          |
| 5                | BusFault                     |                                                                                    |          |
| 6                | UsageFault                   | undefined instruction, ...; configured: division by zero, unaligned address access |          |
| 7-10             | Reserved                     |                                                                                    |          |
| 11               | SVCall                       | SVC                                                                                |          |
| 12               | Debug Monitor                |                                                                                    |          |
| 13               | Reserved                     |                                                                                    |          |
| 14               | PendSV                       | system-level interrupt,                                                            |          |
| 15               | SysTick                      | system-level interrupt, generated by systick timer.                                |          |
| 16               | External Interrupt 0         |                                                                                    |          |
| ...              |                              |                                                                                    |          |
| 16 + N           | External Interrupt N         |                                                                                    |          |

中断向量表的位置, 用 VTOR 寄存器记录 (Vector Table Offset Register).

### 中断模型

## 指令

ARMv7-M 仅支持 THUMB-2 指令集 (32b) 和 THUMB 指令集 (16b), 但都是 16b 对齐的. 

### 控制流指令

- branch, 16b / 32b: B
- cmp & branch, 16b: CBNZ, CBZ
- If-Then following 4 instructions, 16b: IT block 
- call subroutine: BL, BLX (register, 32b), BX (32b)
- TBB, TBH: Table Branch, byte(halfword) offsets.

### 数据指令

- 标准指令
- 移位指令
- 乘法指令
- 饱和算术指令 
- pack/unpack 
- DSP extension: 并行算术指令
- 浮点数指令 (见后)

### 状态指令

### 内存指令 

### 异常处理 

### 协处理器指令

### 浮点数指令

