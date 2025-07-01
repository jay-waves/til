- *张量 (Tensor)*: 多维数组, 一般指三阶及以上.
- 矩阵 (Matrix): 二阶张量.
- 向量 (Vector): 一阶张量.
- 标量 (Scalar): 零阶张量.

## MAC 

Multiply-Accumulate / MAC (dsp) / FMA (gpu), 乘加指令: `mac r1, r2, r2` --> `r1 = r2 * r3 + r1`.  用于向量和矩阵乘加.

## SIMD 

数据并行 (Single Instrution, Multiple Data). 
- 小规模: DSP SIMD, DSP VLIW (Very Long Instruction Word), CPU SEE 
- 大规模: GPU SIMT


## QADD 

溢出时裁剪, 而非 wrap-around. 称为饱和加法.

## 低精度计算

使用低精度浮点数 fp16, fp8 等, 功耗小, 并且吞吐率提升 (空间小). 在神经网络场景下, 低精度几乎不影响模型效果, 但能增大模型规模.