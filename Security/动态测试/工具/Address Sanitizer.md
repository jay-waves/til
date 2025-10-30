Address Sanitizer 是漏洞检查器, 性能损耗最高小于 2.60x, 只写检测平均为 1.26x, 读写检测平均为 1.5x. 内存损耗平均为 3.37x, 最坏为 20x.

主要参数如下:
- depth of stack unwinding: (default 30) 指 `malloc/free` 操作时记录此刻的栈信息, 对调试非常重用. 此操作不额外消耗内存, 但会拖慢速度. 
- quaratine size: (default 256MB) 指 `free` 后的内存隔离区, 大隔离区利于检测 UAF 漏洞.
- size of the heap redzone: (default 128B) 堆内存的红区大小, 红区越大, 对 OOB 检测范围越广, 但性能也会显著降低, 尤其是内存分配比较碎片化的程序.

## 实现原理

### 阴影内存

为了兼容 `double` 等长数据类型, 大多数 64 位平台的 ABI 规定 `malloc` 应返回至少 8 字节对齐的地址, 以满足最大数据类型对齐需求. 因此, ASAN 将内存每 8 字节映射到阴影内存 (Shadow Memory) 的一字节. 给定内存地址 `Addr`, 其对应的阴影内存地址是 `( Addr >> 3 ) + Offset`. 注意, ASAN 忽略了前 `n` 个字节不可访问, 后 `8-n` 个字节可访问的情况; 而总是认为 8 字节内存块的前 `k` 个连续字节是可访问的, 后 `8-k` 个字节是不可访问的. 在现代 C/C++ 语言中, 内存分配总是从低地址开始的.

在逻辑地址中, 假设最大地址空间为 `Max-1`, 那么阴影内存的最大可能体积为 `Max/8`. 因此 `Offset` 选取时, 应至少保证 `Offset ~ Offset+Max/8` 地址部分不被进程占用. 在 32b Unix 平台上, ASAN 使用 `Offset = 0x20000000` $(2^{29})$. 在 64b 平台上, ASAN 使用 `Offset = 0x0000100000000000` $(2^{44})$. 

| 32b                        | 64b                                 | MemoryType |
| -------------------------- | ----------------------------------- | ---------- |
| `[0x40000000, 0xffffffff]` | `[0x10007fff8000, 0x7fffffffffff]`  | HighMem    |
| `[0x28000000, 0x3fffffff]` | `[0x02008fff7000, 0x10007fff7fff]`  | HighShadow |
| `[0x24000000, 0x27ffffff]` | `[0x00008fff7000, 0x02008fff6fff]` | ShadowGap  |
| `[0x20000000, 0x23ffffff]` | `[0x00007fff8000, 0x00008fff6fff]`                                   | LowShadow  |
| `[0x00000000, 0x1fffffff]` | `[00000000000000, 0x00007fff7fff]`                                    | LowMem     |

阴影内存字节的意义如下:
- $k\ (1\leq k\leq 7)$ 指被映射内存前 $k$ 个字节是可用的 (addressable)
- $0$ 指所有 $8$ 个字节都是可用的
- 负值 指所有字节都是不可用的. 不同负值对应不同类型的不可用地址:
	- heap readzones
	- stack redzones
	- global redzones
	- freed memory

```
+--------+
| Memory | -----+
+--------+      |
| Shadow | <----+
+--------+
|  Gap   |
+--------+
| Shadow |
+--------+
| Memory |
+--------+
```

### 插桩逻辑

ASan 修改被测程序, 每当有内存访问时, 先检查对应阴影内存的状态.

```c
//Before:
*address = ...;

//After:
if (SlowPathCheck(address, kAccessSize)) {
	ReportError(address, kAccessSize, kIsWrite);
}
*address = ...;

bool SlowPathCheck(Addr, AccessSize) { // slowpath 指发生内存错误
	ShadowAddr = (Addr >> 3) + Offset;
	k = *ShadowAddr;
	if (k) 
		return ((Addr & 7) + AccessSize > k);
	return False;
}
```

### 运行时

ASAN 运行时部分职责:
- 在程序启动时, ASAN 会初始化阴影内存. 此步的正确性可能会收到 ASLR 的干扰. 
- 拦截动态内存分配函数 `malloc/free` 和栈分配.
- 实时监控内存访问, 非法访问时生成崩溃报告

ASAN 在运行时, 将替换 `malloc, free` 等内存相关调用. 

对于 `malloc` 调用, ASAN 将在正常内存两侧分配隐蔽的红区 (redzone, poinsoned memory), 这部分是被测程序不可访问的, 否则将是 Underflow/Overflow 漏洞. 

对于 `free` 调用, ASan 不会立即将该内存释放, 而是将整个区域标记为已毒化, 防止再次访问 (UAF). 这部分内存会加入一个隔离区 (quarantine), 暂时不允许 malloc 重新分配. 隔离区是一个 FIFO 队列, 队列满时, 才会开始真地释放最早的内存.

红区越大, 可检测范围越广. 在左红区处, 还会存储内存块相关的元数据 (大小, 线程 ID 等). 在调用 `malloc, free` 时, ASan 也会记录"此时的" (将来出错可能是"历史的") 调用栈, `malloc` 调用栈存储在左红区中, 而 `free` 调用栈存储在堆栈上.

```
+-----+------+------+------+-----+------+-----+
| rz1 | mem1 | rez2 | mem2 | rz3 | mem3 | rz4 |
+-----+------+------+------+-----+------+-----+
```


### 栈和全局变量

除了堆内存, ASAn 也支持检测栈和全局变量的内存缺陷. 对于全局变量, 红区将在编译时被创建, 其地址在运行起始时被递送到运行时库; 运行时库函数负责监控这些红区. 对于栈对象, 红区在运行时被创建, 每个红区占 32B 大小. 

```c
void foo() {
	char a[10];
	...
}

void foo() { // poisoned
	char rz1[32];     // 32-bytes aligned
	char arr[10];   
	char rz2[32-10];  // 32-bytes aligned
	char rz3[32];     // 32-bytes aligned
	unsigned *shadow = (unsigned*) (((long)rz1 >> 8) + Offset);
	// poison redzones around arr
	shadow[0] = 0xffffffff; // rz1
	shadow[1] = 0xffff0200; // rz2, unpoison arr
	shadow[2] = 0xffffffff; // rz3
	...
	// un-poison all.
	shadow[0] = shadow[1] = shadow[2] = 0;
}
```

### 错误报告

可以通过函数实现, 也可以用硬件指令. 如 `ud2` 是 x86 中的未定义指令, 会触发 `SIGILL` 信号使程序崩溃. 难点是如何传递尽可能多的信息, 同时保证体积尽可能小, 避免导致程序体积膨胀或受干扰.

```assembly
mov %rax, [addres]
ud2
db error_info
```

程序崩溃时, 可能只能获得一个指令地址及少量信息. 如代码详细位置和调用栈等, 需要用到*地址到符号解析技术*. 编译时使用 `-g` 保留调试符号信息 (DWARF), 然后结合反汇编程序解析符号表. ASan 将执行下列步骤:
1. 捕获崩溃地址
2. 通过影子内存查找该地址, 确定其内存不可访问类型.
3. 通过地址反解析符号.
4. 获取完整调用栈, 并打印.

一般通过 glibc execinfo.h 中的 `backtrace` 接口获取调用栈, 其会逐层遍历栈帧从而解析整个调用栈. 也可以用 libunwind 库.

### 健全性和完整性

ASAN 没有假阳漏洞, 即不会有误报, 当然有一些低价值漏洞. 可能由假阴漏洞, 即遗漏漏洞. ASAN 的插桩操作应该在编译时的最后步骤进行, 避免一些编译器优化行为的干扰.

ASAN 插桩可能会遗漏一种漏洞: 越界的未对齐的地址访问. 这种漏洞比较罕见, 纳入检测会拖慢效率, ASAN 忽略了该漏洞. 这种非对齐访问可以用静态检测检查.

```cpp
int *a = new int[2]; // 8-aligned
int *u = (int*)((char*)a + 6);
*u = 1; // Access to rang [6-9]
```

当错误访问的地址过远, 恰好越过了红区, 命中了另一个有效内存, 也可能导致 OOB 漏报.

```cpp
char *a = new char[100];
char *b = new char[1000];
a[500] = 0; // 会访问 b 的某处
```

被释放的内存会被 ASAN 标记为红区, 但如果其被重新分配给其他变量, 可能导致 UAF 漏报. 栈释放后使用 (UAR) 也是该情况.

```cpp
char *a = new char[1 << 20]; // 1MB
delete []a;                  // free
char *b = new char[1 << 28]; // 256MB
delete []b;                  // drains the quaratine queue
char *c = new char[1 << 20]; // 1MB
a[0] = 0;                    // "use after free", may land in c
```

在理论上, ASAN 是线程安全的. 魔改的运行时 `malloc/free` 函数会使用线程本地缓存, 来避免每次调用时的锁问题. ASan 可能检测到 DataRace, 比如删除和读取操作的竞争被识别为 UAF, 但并不保证.

## in cpp

### 插桩阶段 (Instrumentation Pass)

ASAN 实现代码在 `/llvm-project/llvm/lib/Transforms/Instrumentation/AddressSanitizer.cpp`. FunctionPass 遍历所有函数进行插桩 `instrumentFunciton()`

```cpp
bool AddressSanitizer::instrumentFunction(Function &F,const TargetLibraryInfo *TLI) {
...
 // 这些Vector用于保存筛选出来的指令对象和信息
  SmallVector<InterestingMemoryOperand, 16> OperandsToInstrument;
  SmallVector<MemIntrinsic *, 16> IntrinToInstrument;
  SmallVector<BasicBlock *, 16> AllBlocks;
  int NumAllocas = 0;

  for (auto &BB : F) {
    AllBlocks.push_back(&BB);
    for (auto &Inst : BB) {
      SmallVector<InterestingMemoryOperand, 1> InterestingOperands;
      getInterestingMemoryOperands(&Inst, InterestingOperands);

// 如果当前指令属于需要插桩的位置,那就记录一下,后面会用到
      if (!InterestingOperands.empty()) {  
        for (auto &Operand : InterestingOperands) {
          OperandsToInstrument.push_back(Operand);
        }
      } else if (MemIntrinsic *MI = dyn_cast<MemIntrinsic>(&Inst)) {  
	    // memset/memcpy/memmove
        IntrinToInstrument.push_back(MI);
      }
    }
  }
...
// 对数据访问指令进行操作
  for (auto &Operand : OperandsToInstrument) {  
    instrumentMop(ObjSizeVis, Operand, UseCalls,
                    F.getParent()->getDataLayout());
    FunctionModified = true;
  }
// 对内存操作指令进行操作
  for (auto Inst : IntrinToInstrument) {  
    instrumentMemIntrinsic(Inst);
    FunctionModified = true;
  }

// 对插桩之后的函数进行栈调整
  FunctionStackPoisoner FSP(F, *this);
  bool ChangedStack = FSP.runOnFunction(); 
    
...
  return FunctionModified;
}

void AddressSanitizer::getInterestingMemoryOperands(
    Instruction *I, SmallVectorImpl<InterestingMemoryOperand> &Interesting) {
// LLVM IR Load指令
  if (LoadInst *LI = dyn_cast<LoadInst>(I)) {  
// 判断指令中的操作数是否为指针
    if (ignoreAccess(LI->getPointerOperand()))  
      return;
    Interesting.emplace_back(I, LI->getPointerOperandIndex(), false,
                             LI->getType(), LI->getAlign());
  } 
// LLVM IR Store指令,用于保存数据
  else if (StoreInst *SI = dyn_cast<StoreInst>(I)) {  
    if (ignoreAccess(SI->getPointerOperand()))
      return;
    Interesting.emplace_back(I, SI->getPointerOperandIndex(), true,
                             SI->getValueOperand()->getType(), SI->getAlign());
  }
}
```

筛选出需要插桩的指令后, 在 `LLVM IR Load/Store` 指令前插入异常检测逻辑, 即如果没有异常才能够执行原本的读写操作, 否则 ASAN 报告异常.

```cpp
void AddressSanitizer::instrumentMop(
									ObjectSizeOffsetVisitor &ObjSizeVis,
									InterestingMemoryOperand &O, 
									bool UseCalls,const DataLayout &DL
) {
// 获取指令操作的指针地址
  Value *Addr = O.getPtr();  
...
// 内存检测粒度,后续再详解
  unsigned Granularity = 1 << Mapping.Scale;  
    
  doInstrumentAddress(this, O.getInsn(), O.getInsn(), Addr, O.Alignment,
                  Granularity, O.TypeSize, O.IsWrite, nullptr, UseCalls,
                  Exp);
}

static void doInstrumentAddress(AddressSanitizer *Pass, Instruction *I,
                                Instruction *InsertBefore, Value *Addr,
                                MaybeAlign Alignment, unsigned Granularity,
                                uint32_t TypeSize, bool IsWrite,
                                Value *SizeArgument, bool UseCalls,
                                uint32_t Exp
) {
// 若当前指令访问方式是按 8 字节对齐的 (Shadow 映射默认是 8 字节对齐)
  if ((TypeSize == 8 || TypeSize == 16 || TypeSize == 32 || TypeSize == 64 ||
       TypeSize == 128) &&
      (!Alignment || *Alignment >= Granularity || *Alignment >= TypeSize / 8))
    return Pass->instrumentAddress(I, InsertBefore, Addr, TypeSize, IsWrite,
                                   nullptr, UseCalls, Exp);  
  Pass->instrumentUnusualSizeOrAlignment(I, InsertBefore, Addr, TypeSize,
                                         IsWrite, nullptr, UseCalls, Exp);
}

void AddressSanitizer::instrumentAddress(Instruction *OrigIns,Instruction *InsertBefore, Value *Addr,uint32_t TypeSize, bool IsWrite,Value *SizeArgument, bool UseCalls,uint32_t Exp) {
  bool IsMyriad = TargetTriple.getVendor() == llvm::Triple::Myriad;

  IRBuilder<> IRB(InsertBefore);  // LLVM IR 指令生成器
  Value *AddrLong = IRB.CreatePointerCast(Addr, IntptrTy);
  size_t AccessSizeIndex = TypeSizeToSizeIndex(TypeSize);

  Type *ShadowTy =
      IntegerType::get(*C, std::max(8U, TypeSize >> Mapping.Scale));
  Type *ShadowPtrTy = PointerType::get(ShadowTy, 0);
  Value *ShadowPtr = memToShadow(AddrLong, IRB);
  Value *CmpVal = Constant::getNullValue(ShadowTy);
  Value *ShadowValue =
      IRB.CreateLoad(ShadowTy, IRB.CreateIntToPtr(ShadowPtr, ShadowPtrTy));

  Value *Cmp = IRB.CreateICmpNE(ShadowValue, CmpVal);
  Instruction *CrashTerm = nullptr;

/*
上面这段指令生成的意思是创建if判断:
shadow_page_flag = 
	*(_BYTE *)((((unsigned __int64)real_data + 0x1001) >> 3) + 0x7FFF8000);
real_data_offset = (unsigned __int64)real_data + 0x1001;
if ( shadow_page_flag )  // ASAN内存异常检测插桩判断
*/
    
  CrashTerm = SplitBlockAndInsertIfThen(Cmp, InsertBefore, !Recover);
  Instruction *Crash = generateCrashCode(CrashTerm, AddrLong, IsWrite, AccessSizeIndex, SizeArgument, Exp);
    
/*
上面这段指令生成的意思是 if 判断成功时,在它的子 BasicBlock 中创建报错用函数调用:
_asan_report_store1(v13);

所以合并起来插桩代码就是:
shadow_page_flag = 
	*(_BYTE *)((((unsigned __int64)real_data + 0x1001) >> 3) + 0x7FFF8000);
real_data_offset = (unsigned __int64)real_data + 0x1001;
if ( shadow_page_flag ) 
	_asan_report_store1(real_data_offset); 
*/
}
```

调整函数的栈空间, 把 ShadowTable 的分配和销毁引入进来.

```cpp
bool AddressSanitizer::instrumentFunction() {
...
   FunctionStackPoisoner FSP(F, *this);
   bool ChangedStack = FSP.runOnFunction();
...
}

bool runOnFunction() {
...
// 遍历所有指令,筛选出内存分配操作
   for (BasicBlock *BB : depth_first(&F.getEntryBlock())) visit(*BB);
...
   processDynamicAllocas();
   processStaticAllocas();
...
   return true;
}

// 处理 Alloc 指令
void visitAllocaInst(AllocaInst &AI) {

//当前函数所有的变量声明, 都会同一将其内存申请移至函数入口的BB中.
//isStaticAlloca() 判断 Alloc 是否在函数入口BB执行 + 判断该内存大小是否会变
  if (!AI.isStaticAlloca()) 
    DynamicAllocaVec.push_back(&AI);
  else
    AllocaVec.push_back(&AI);
}

void visitIntrinsicInst(IntrinsicInst &II) {
  bool DoPoison = (ID == Intrinsic::lifetime_end);
  AllocaPoisonCall APC = {&II, AI, SizeValue, DoPoison};
  if (AI->isStaticAlloca()) 
    StaticAllocaPoisonCallVec.push_back(APC);  // 记录栈中分配对象大小和偏移信息
  else if (ClInstrumentDynamicAllocas)
    DynamicAllocaPoisonCallVec.push_back(APC);
}
```

```cpp
void FunctionStackPoisoner::processStaticAllocas() {
  // 在第一个 Alloc 前插桩
  Instruction *InsBefore = AllocaVec[0]; 
  IRBuilder<> IRB(InsBefore);

  SmallVector<ASanStackVariableDescription, 16> SVD;
  SVD.reserve(AllocaVec.size());
  for (AllocaInst *AI : AllocaVec) { 
    ASanStackVariableDescription D = {AI->getName().data(),
        ASan.getAllocaSizeInBytes(*AI), 0, AI->getAlignment(), AI, 0, 0};
    SVD.push_back(D);
  }

  size_t Granularity = 1ULL << Mapping.Scale;  // 内存粒度
  size_t MinHeaderSize = std::max((size_t)ASan.LongSize / 2, Granularity);
  // 调整后的栈布局
  const ASanStackFrameLayout &L =
      ComputeASanStackFrameLayout(SVD, Granularity, MinHeaderSize);
  uint64_t LocalStackSize = L.FrameSize;

// 调整新栈的空间
  Value *StaticAlloca =
      DoDynamicAlloca ? nullptr : createAllocaForLayout(IRB, L, false); 
  Value *FakeStack;
  Value *LocalStackBase;
  Value *LocalStackBaseAlloca;
  uint8_t DIExprFlags = DIExpression::ApplyOffset;

  LocalStackBaseAlloca =
      IRB.CreateAlloca(IntptrTy, nullptr, "asan_local_stack_base");
  Constant *OptionDetectUseAfterReturn = F.getParent()->getOrInsertGlobal(
      kAsanOptionDetectUseAfterReturn, IRB.getInt32Ty());
  Value *UseAfterReturnIsEnabled = IRB.CreateICmpNE(
      IRB.CreateLoad(IRB.getInt32Ty(), OptionDetectUseAfterReturn),
      Constant::getNullValue(IRB.getInt32Ty()));
  Instruction *Term =
      SplitBlockAndInsertIfThen(UseAfterReturnIsEnabled, InsBefore, false);
  IRBuilder<> IRBIf(Term);
  StackMallocIdx = StackMallocSizeClass(LocalStackSize);
  assert(StackMallocIdx <= kMaxAsanStackMallocSizeClass);
  Value *FakeStackValue =
      IRBIf.CreateCall(AsanStackMallocFunc[StackMallocIdx],
                       ConstantInt::get(IntptrTy, LocalStackSize));
  IRB.SetInsertPoint(InsBefore);
  FakeStack = createPHI(IRB, UseAfterReturnIsEnabled, FakeStackValue, Term,
                        ConstantInt::get(IntptrTy, 0));

  Value *NoFakeStack =
      IRB.CreateICmpEQ(FakeStack, Constant::getNullValue(IntptrTy));
  Term = SplitBlockAndInsertIfThen(NoFakeStack, InsBefore, false);
  IRBIf.SetInsertPoint(Term);
  Value *AllocaValue =
      DoDynamicAlloca ? createAllocaForLayout(IRBIf, L, true) : StaticAlloca;

  IRB.SetInsertPoint(InsBefore);
  LocalStackBase = createPHI(IRB, NoFakeStack, AllocaValue, Term, FakeStack);
  IRB.CreateStore(LocalStackBase, LocalStackBaseAlloca);
  
/* 
生成的插桩代码等价于:
void *FakeStack = __asan_option_detect_stack_use_after_return
	? __asan_stack_malloc_N(LocalStackSize) : nullptr;
void *LocalStackBase = (FakeStack) ? FakeStack : alloca(LocalStackSize);

从ShadowTable中分配一块栈内存, 用于异常检测的. 
__asan_stack_malloc_N() 在Compiler-RT 中实现
*/

// 获取 ShadowTable 中的栈起始地址
  Value *LocalStackBaseAllocaPtr = isa<PtrToIntInst>(LocalStackBaseAlloca)
          ? cast<PtrToIntInst>(LocalStackBaseAlloca)->getPointerOperand()
          : LocalStackBaseAlloca;  

// 根据 AllocaInst 的申请栈分配内存大小和位置,在 ShadowTable 中重新调整位置
  for (const auto &Desc : SVD) {  
    AllocaInst *AI = Desc.AI;
    Value *NewAllocaPtr = IRB.CreateIntToPtr(
        IRB.CreateAdd(LocalStackBase, ConstantInt::get(IntptrTy, Desc.Offset)),
        AI->getType());
    AI->replaceAllUsesWith(NewAllocaPtr);
  }

// 不重要,在 ShadowTable 中记录当前函数的信息
  Value *BasePlus0 = IRB.CreateIntToPtr(LocalStackBase, IntptrPtrTy);
  IRB.CreateStore(ConstantInt::get(IntptrTy, kCurrentStackFrameMagic),
                  BasePlus0);
  // Write the frame description constant to redzone[1].
  Value *BasePlus1 = IRB.CreateIntToPtr(
      IRB.CreateAdd(LocalStackBase,
                ConstantInt::get(IntptrTy, ASan.LongSize / 8)), IntptrPtrTy);
  GlobalVariable *StackDescriptionGlobal =
      createPrivateGlobalForString(*F.getParent(), DescriptionString,
                                   /*AllowMerging*/ true, kAsanGenPrefix);
  Value *Description = IRB.CreatePointerCast(StackDescriptionGlobal, IntptrTy);
  IRB.CreateStore(Description, BasePlus1);
  // Write the PC to redzone[2].
  Value *BasePlus2 = IRB.CreateIntToPtr(
	    IRB.CreateAdd(LocalStackBase,
                ConstantInt::get(IntptrTy, 2 * ASan.LongSize / 8)), IntptrPtrTy);
  IRB.CreateStore(IRB.CreatePointerCast(&F, IntptrTy), BasePlus2);

// 根据 SVD 中记录栈中各个变量对应的内存位置初始化 ShadowTable 的栈内存
  const auto &ShadowAfterScope = GetShadowBytesAfterScope(SVD, L);  

// ASan.memToShadow() 用于计算进程内存在 ShadowTable 的偏移位置
  Value *ShadowBase = ASan.memToShadow(LocalStackBase, IRB);  
// 1. 给函数栈内存投毒
  copyToShadow(ShadowAfterScope, ShadowAfterScope, IRB, ShadowBase);

// 2.对栈中分配的变量在 ShadowTable 中消毒
  if (!StaticAllocaPoisonCallVec.empty()) {  
    const auto &ShadowInScope = GetShadowBytes(SVD, L);

    for (const auto &APC : StaticAllocaPoisonCallVec) {
      const ASanStackVariableDescription &Desc = *AllocaToSVDMap[APC.AI];
      assert(Desc.Offset % L.Granularity == 0);
      size_t Begin = Desc.Offset / L.Granularity;
      size_t End = Begin + (APC.Size + L.Granularity - 1) / L.Granularity;

      IRBuilder<> IRB(APC.InsBefore);
      copyToShadow(ShadowAfterScope,
                   APC.DoPoison ? ShadowAfterScope : ShadowInScope, Begin, End,
                   IRB, ShadowBase);
    }
  }
  
  /*
  投毒再消毒后, ShadowTable 的内存数据布局如下:
  1. ShadowTable 分配栈后对内存投毒 =>  F3F3F8F8F1F1F1F1
  2. 对栈中需要用到的变量位置消毒     =>  F3F30000F1F1F1F1
  此时访问栈变量,获取到的数据就是 0x00, 为正常数据访问
  */
    
  SmallVector<uint8_t, 64> ShadowClean(ShadowAfterScope.size(), 0);
  SmallVector<uint8_t, 64> ShadowAfterReturn;

  for (auto Ret : RetVec) {
    IRBuilder<> IRBRet(Ret);
    // Mark the current frame as retired.
    IRBRet.CreateStore(ConstantInt::get(IntptrTy, kRetiredStackFrameMagic),
                       BasePlus0);
 
//简单总结就是在函数返回时清空 ShadowTable 中的栈数据为 0xF5
	// if FakeStack != 0  // LocalStackBase == FakeStack
	//     // In use-after-return mode, poison the whole stack frame.
	//     if StackMallocIdx <= 4
	//         // For small sizes inline the whole thing:
	//         memset(ShadowBase, kAsanStackAfterReturnMagic, ShadowSize);
	//         **SavedFlagPtr(FakeStack) = 0
	//     else
	//         __asan_stack_free_N(FakeStack, LocalStackSize)
	// else
	//     <This is not a fake stack; unpoison the redzones>

    Value *Cmp =
        IRBRet.CreateICmpNE(FakeStack, Constant::getNullValue(IntptrTy));
    Instruction *ThenTerm, *ElseTerm;
    SplitBlockAndInsertIfThenElse(Cmp, Ret, &ThenTerm, &ElseTerm);

    IRBuilder<> IRBPoison(ThenTerm);
    if (StackMallocIdx <= 4) {
      int ClassSize = kMinStackMallocSize << StackMallocIdx;
      ShadowAfterReturn.resize(ClassSize / L.Granularity,
                               kAsanStackUseAfterReturnMagic);
      copyToShadow(ShadowAfterReturn, ShadowAfterReturn, IRBPoison,
                   ShadowBase);
      Value *SavedFlagPtrPtr = IRBPoison.CreateAdd(
          FakeStack,
          ConstantInt::get(IntptrTy, ClassSize - ASan.LongSize / 8));
      Value *SavedFlagPtr = IRBPoison.CreateLoad(
          IntptrTy, IRBPoison.CreateIntToPtr(SavedFlagPtrPtr, IntptrPtrTy));
      IRBPoison.CreateStore(
          Constant::getNullValue(IRBPoison.getInt8Ty()),
          IRBPoison.CreateIntToPtr(SavedFlagPtr, IRBPoison.getInt8PtrTy()));
    } else {
      // For larger frames call __asan_stack_free_*.
      IRBPoison.CreateCall(
          AsanStackFreeFunc[StackMallocIdx],
          {FakeStack, ConstantInt::get(IntptrTy, LocalStackSize)});
    }

    IRBuilder<> IRBElse(ElseTerm);
    copyToShadow(ShadowAfterScope, ShadowClean, IRBElse, ShadowBase);
  }
}
```

### 运行时逻辑 (Compiler-RT)

ASan 有大量内存分配/操作功能, 为了避免 Pass 过于臃肿, 将一些核心功能写在了 Compiler-RT 中, 让 Clang 在链接阶段引入它们.

```cpp
void __asan_init() {
  AsanActivate();
  AsanInitInternal();
}

void AsanActivate() {
// 从环境变量ASAN_ACTIVATION_OPTIONS中获取ASAN配置
  asan_deactivated_flags.OverrideFromActivationFlags();  

  SetCanPoisonMemory(asan_deactivated_flags.poison_heap);
  SetMallocContextSize(asan_deactivated_flags.malloc_context_size);
  ReInitializeAllocator(asan_deactivated_flags.allocator_options);
}

static void AsanInitInternal() {
  InitializeHighMemEnd();
  InitializeShadowMemory();

  AllocatorOptions allocator_options;
  allocator_options.SetFrom(flags(), common_flags());
  InitializeAllocator(allocator_options);

  InitializeCoverage(common_flags()->coverage, common_flags()->coverage_dir);
}
```

### 参考

[AddressSanitizerCompileTimeOptimizations](https://github.com/google/sanitizers/wiki/AddressSanitizerCompileTimeOptimizations)

Source and Fuzzing. Icatro. [深入解析 libfuzzer 与 asan 原理](https://github.com/lcatro/Source-and-Fuzzing/blob/35f45172497519cf4306f2649f0e4679cf8cee0a/12.%E6%B7%B1%E5%85%A5%E8%A7%A3%E6%9E%90libfuzzer%E4%B8%8Easan.md#asan%E5%8E%9F%E7%90%86).

AddressSanitizer: A Fast Address Sanity Checker. Knostantin Serebryany, Derek Bruening, Alexander Potapenko, Dmitry Vyukov, Google. USENIX ATC 2012. @konstantin2012

[Address Sanitizer Algorithm -- Github Wiki](https://github.com/google/sanitizers/wiki/AddressSanitizerAlgorithm)