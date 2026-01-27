CUDA 的硬件架构指令集有: Tesla, Pascal, Volta, Turing, Ampere, Hopper, Blackwell 等


GPU 的分类：
* GPGPU (General Purpose GPU). 图形处理、高精度浮点数运算（64b）。
* NPU. 张量（矩阵）计算，低精度数值计算（INT8，BF16）。


## Ampere 

CUDA 是用户可编程语言, 通过一系列步骤翻译为 GPU 指令:

```
CUDA: Compute Unified Device Architecture. 用户层.

 |   nvcc 编译器
 v

PTX: Parallel Thread Execution. PTX 指令, 有详细文档, 支持向后兼容.

 |  ptxas 汇编器
 v

SASS: Shader ASSembly. SASS 指令直接由 GPU 执行, 这部分基本闭源.
```

### 内存模型

1. 全局内存 global memory 
2. 共享内存 shared memory 
3. 寄存器

### 寄存器

Nvidia GPU 的基本调度单位为*线程束 (warp)*, 每个线程束包含多个*线程 (lane)*, 线程共享同样的代码但有私有的通用寄存器组. 

Nvidia GPU 的特殊寄存器一般是只读的, 标识执行单元的定位信息: 寄存器是 32b 位宽.
- `SR_TID` : threadIdx, 
- `SR_CTAID`: blockIdx
- `SR_CLOCKLO, SR_CLOCKHI`: 
- `SR_VIRTUALSMID, SR_LANEID, SR_LEMASK, SR_LTMASK, SR_GEMASK`

Prediction Registers 用于分支预测, 每个线程私有. 
1. 作为指令的执行条件, 放在指令开始: `@P6 FADD R5 R5 R28`
2. 作为操作数参与运算: `FMNMX R9 RZ R6 !PT`

Uniform 寄存器是 warp 中所有 lanes 共享的公共寄存器.

Predication 寄存器

常量 Cache 

### 指令

整数操作指令

浮点操作指令 

BIT 操作和逻辑指令

特殊函数指令

分支和控制指令

数据加载和存储指令

Warp Level 指令

Atomic 指令

#### 访存指令

| 指令 | 类型  | 目标位置 | 源位置   |
| ---- | ----- | -------- | -------- |
| LDG  | LOAD  | 寄存器   | 全局内存 |
| STG  | STORE | 全局内存 | 寄存器   |
| LDS  | LOAD  | 寄存器   | 共享内存 |
| STS  | STORE | 共享内存 | 寄存器   |
| LDL  | LOAD  | 寄存器   | 局部内存 |
| STL  | STORE | 局部内存 | 共享内存 |
| LDSM | LOAD  | 寄存器   | 共享内存         |


## 参考

https://zhuanlan.zhihu.com/p/686198447. reed. 系列专栏文章.