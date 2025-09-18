## 对象关系

```
Value 
+-- Argument 
+-- User 
	+-- Constant 
		+-- Constant Expr 
		+-- Constant Data 
			+-- ConstantInt 
			+-- UndefValue 
	+-- Operator 
		+-- ConcreteOperator 
	+-- Instruction 
		+-- UnaryInst 
		+-- Binary Operator 
		+-- CallBase
	+-- Function 
	+-- GlobalVariable 
```

User 是 `Value` 的一个子类, 表示需要引用其他 `Value` 作为操作数的对象. `User` 内部有一个 use-list, 记录了其引用的所有 `Value`. 

Llvm 认为 `Value` 间的引用是**有向无环图**. 通过 SSA 和无环控制流, 保证这一点.

简单的遍历程序:

```cpp 
#include "llvm/IR/Module.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm; 


// 创建 LLVM 上下文和模块
LLVMContext Context;
Module *M = new Module("MyModule", Context);

// 按结构遍历
for (Function &F : *M) {

	for (BasicBlock &BB : F) {

		for (Instruction &I : BB) {

				CallInst *CI = dyn_cast<CallInst>(I);
				UnaryInstruction *UI = dyn_cast<UnaryInst>(I);
				CastInst *CI = dyn_cast<CastInst>(I);
				
				for (Op &O : I.operands()) 
					Value *Op = I.getOperand();
				
				
				for (Use &U : I.uses()) 
					Value *UserVal = U.getUser(); 
				
		}
	}
}
```

## CFG 

```cpp 
// 按 CFG 遍历
for (BasicBlock &BB : *F) {
	Instruction *Term = BB.getTerminator();
	for (unsigned i = 0; i < Term->getNumSuccessors(); ++i) {
		BasicBlock *Succ = Term->getSuccessor(i);
	}
}

// 遍历基本块的前驱
BB.pred_begin(); BB.pred_end();

// 遍历基本块的后继
BB.succ_begin(); BB.succ_end();

// 支配树
#include "llvm/Analysis/DominatorTree.h"
DominatorTree DT(*F);

// 循环结构
#include "llvm/Analysis/LoopInfo.h"
LoopInfo(DT);
```