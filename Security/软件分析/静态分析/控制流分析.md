CFG Analysis:
- Dominator Tree Analysis  / PostDominator
- Loop Info Analysis 
- Branch ProbabilityInfo
- AA Analysis 
- LazyValue Info 
- MemorySSA 

## 基本块

### 划分基本块

**识别所有基本块的起始**:
- 程序第一个指令
- 任意跳转指令的目标指令
- 任意跳转指令的后续指令

## 控制流

### Dominator Tree 

找到那些块一定在执行某块之前

### Loop Analysis 

循环头, 循环体, 反向边

### Path-sensitive analysis