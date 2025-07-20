- *张量 (Tensor)*: 多维数组, 一般指三阶及以上.
- 矩阵 (Matrix): 二阶张量.
- 向量 (Vector): 一阶张量.
- 标量 (Scalar): 零阶张量.

DSP 一般使用*哈弗架构*, 将数据总线和指令总线分开. 在 DSP 核心附近会有紧耦合内存 (TCM), 用于缓存预取数据.

## MAC 

Multiply-Accumulate / MAC (dsp) / FMA (gpu), 乘加指令: `mac r1, r2, r2` --> `r1 = r2 * r3 + r1`.  用于向量和矩阵乘加.


## SIMD 

SIMD 是向量化指令, 包括向量化访存和向量化计算. 

数据并行 (Single Instrution, Multiple Data):
- 小规模: DSP SIMD, DSP VLIW (Very Long Instruction Word), Qualcomm HVX, ARM NEON. 
- 大规模: GPU SIMT

### VLIW

超长指令字, 指令包含多个指令槽 (Slot), 相当于一次发射并行多条指令. 是 DSP 上常见的 SIMD 指令类型.


## QADD 

溢出时裁剪, 而非 wrap-around. 称为饱和加法.

## 低精度计算

使用低精度浮点数 fp16, fp8 等, 功耗小, 并且吞吐率提升 (空间小). 在神经网络场景下, 低精度几乎不影响模型效果, 但能增大模型规模.