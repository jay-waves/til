## Passes 

`-mem2reg`: 将内存操作优化为寄存器

`-loops`: 循环分析

`-simplifycfg`: 简化 CFG, 合并无分支的基本块 / 删除不可达的基本块. 

`-basicaa, -aa-eval, -scev-aa` 别名分析. 

`-dce` 代码不可达分析

`-sccp`: sparse conditional constant propagation. (aggressively)

`-licm` move code out of loops where possible 

`-loop-simplify` put loop struturers in standard form?

CFG (Loop, SCEV, LCSSA) --> SCCP --> AAManager / MemorySSA / LVI --> Interprocedural Taint Analysis

## Ref

 https://llvm.org/docs/WritingAnLLVMNewPMPass.html