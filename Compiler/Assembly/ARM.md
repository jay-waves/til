ARM 是一种精简指令计算 (RISC, Reduced Instruction Set Computing) 架构. 使用 ARM 架构的设备功耗低, 如苹果笔记本续航远超 windows-x86. 
ARM 指令集执行效率高, 即每条指令占用的 CPU 周期较少, 但代码密度相对低一些. RISC 指令比之 CISC 主要为试图保持 `opcode` 长度相等.

ARM 指令集合可以分为三类: ARM, Thumb, ARM64. 
- ARM 为 32 位
- Thumb 为 16 或 32 位. 

ARM 指令格式:
```asm
<op_code> <dest_reg>, <src_reg1>, <src_reg2|imme_num>
```

ARM 支持三种处理器模式:
- 用户模式, User Mode
- 系统模式, System Mode
- 中断模式, Interrupt Mode

## 版本

- ARMv7, 2009. 32b, ARMv7-A 架构, 支持 Thumb-2 指令集, 以及 NEON SIMD / VFPv3 浮点单元
- ARMv8, 2011, 也叫 AArch64. 支持 64b, 引入 TrustZone, 虚拟化指令 
- ARMv9, 2021. 支持 SVE2 向量扩展指令. 

Cortex 系列:
- Cortex-A: ARMv7-A, v8-A, v9-A. 高性能处理器. 有 MMU 单元和虚拟内存支持, 主要用于移动端.
- Cortex-R: ARMv7-R, v8-R. 32b, 用于各类实时系统 RTOS. 不常见.
- Cortex-M: ARMv6-M, v7-M, v8-M. 32b 低功耗处理器, 用于嵌入式. 多指 Thumb 系列紧凑指令集.

Neoverse 系列: 64b, 用于服务器和数据中心.
- 用于 ARM 


## 汇编

### 数据类型

```asm
ldr         # Load word (32b)
ldrh        # Load unsigned half word (16b)
ldrsh       # load signed half word 
ldrb        # load unsigned byte (8b)
ldrsb       # load signed bytes
```

### 寻址方式

1. 立即寻址 `mov r0, #0x1f`
2. 寄存器寻址 `mov r0, r1`
3. 寄存器移位寻址 `mov r0, r1, lsl #2`
	- `LSL` 逻辑左移, 空位补 `0`
	- `LSR` 逻辑右移
	- `ASR` 算数右移, 符号位不变
	- `ROR` 循环右移
4. 寄存器间接寻址: `ldr r0, [r1]`, 类似 `r0 = *r1`
5. 基址寻址 `ldr r0, [r1, #-4]`, 类似 `r0 = *(r1-4)`
6. 


### 指令

数据处理指令:
```asm
MOV R0, #5       ; 将立即数 5 移动到寄存器 R0 
ADD R0, R1, R2   ; R0 = R1 + R2
SUB R0, R1, R2   ; R0 = R1 - R2
MUL 

LSL              ; Logical Shift Left
LSR              ; Logical Shift Right
ASR              ; Arithmetic Shift Right
ROR              ; Rotate Right
CMP            

AND              ; bitwise and
ORR              ; bitwise or
EOR              ; bitwise xor
```

数据传输指令:
```asm
LDR R0, [R1]     ; 用 R1 取址内存, 加载到寄存器 R0
STR R0, [R1]     ; 将寄存器 R0 中值, 保存到 R1 指向的内存地址
LDM              ; load muptiple
STM

PUSH
POP
```

控制流指令:
```asm
B label          ; 无条件跳转到标签 label
BL subroutine    ; branch with link, 跳转到子程序并保存 (链接) 返回地址
BX               ; branch with eXchange
BLX             

SWI/SVC          ; system call
```

条件执行: `EQ`, `NE`, `GT`
```asm
MOVEQ R0, #1       ; 如果上一个比较结果为相等, 则将 1 移动到寄存器 R0
```


### 寄存器

ARM 有 16 个 32 位通用寄存器 (R0-R15)
- `R0-R11` 通用寄存器
- `R12` IP, Intra Procedural Call
- `R13` SP, 堆栈指针
- `R14` LR, 链接寄存器, 保存子程序返回地址
- `R15` PC, 程序计数器, 保存当前指令的地址

专用寄存器:
- CPSR, Current Program Status Register, 程序状态寄存器.

| N        | Z    | C     | V        | Q         |     | J       |     | GE  |     | E          | A             | I           | F           | T     | M   |
| -------- | ---- | ----- | -------- | --------- | --- | ------- | --- | --- | --- | ---------- | ------------- | ----------- | ----------- | ----- | --- |
| negative | zero | carry | overflow | underflow |     | Jazelle |     | Greater than or Equal for SIMD    |     | Endianness | Abort disable | IRQ disable | FIQ disable | Thumb | processor mode <br> privilege mode    |


ARM **调用约定**中规定:
- `R0` 传递返回值
- `R11 (FP, Frame Pointer)` 存储系统函数的调用值, 也就是函数的栈帧
- `R13 (SP)` 存储栈顶
- `R14 (LR)` 存储函数的返回值
- `R15 (PC)` 当前指令地址 (自动增加)


> https://chan-shaw.github.io/2020/03/20/arm%E6%B1%87%E7%BC%96%E8%AF%AD%E8%A8%80%E5%AD%A6%E4%B9%A0%E7%AC%94%E8%AE%B0/
> https://github.com/JnuSimba/AndroidSecNotes/blob/master/Android%E9%80%86%E5%90%91%E5%9F%BA%E7%A1%80/ARM%20%E6%B1%87%E7%BC%96%E6%8C%87%E4%BB%A4%E7%AE%80%E4%BB%8B.md
