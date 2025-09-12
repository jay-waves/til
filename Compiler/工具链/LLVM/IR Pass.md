
## Pass in Src Tree

**[using new pass manager](use%20new%20pass%20manager.md)**

### build your llvm

```shell
cmake -G Ninja ../llvm \
  -DCMAKE_BUILD_TYPE=MinSizeRel \
  -DLLVM_ENABLE_PROJECTS="clang;compiler-rt" \
  -DLLVM_TARGETS_TO_BUILD="X86" \
  -DLLVM_BUILD_EXAMPLES=OFF \
  -DLLVM_BUILD_TESTS=OFF

ninja
```

注意, 如果使用 IDE, 配置 `includePath` 时, 需要同时添加两个目录:
- `llvm-project/llvm/include`
- `llvm-project/build/include`, 其中 build 是 CMake 的构建目录. 这是因为有些头文件在构建时才产生.

### add file to cmake

在 `llvm/lib/Transforms/Utils` 下活动, 修改 `.../Utils/CMakeLists.txt`

### create pass

从 LLVM 9.0 版本开始, 引入了新 Pass Manager, API 有所改变, 并且注册方式也有更改.

```cpp
//../Utils/HelloWorld.h: 

#ifndef LLVM_TRANSFORMS_HELLONEW_HELLOWORLD_H
#define LLVM_TRANSFORMS_HELLONEW_HELLOWORLD_H

#include "llvm/IR/PassManager.h"

namespace llvm {

class HelloWorldPass : public PassInfoMixin<HelloWorldPass> {
public:
  PreservedAnalyses run(Function &F, FunctionAnalysisManager &AM);
};

} // namespace llvm

#endif // LLVM_TRANSFORMS_HELLONEW_HELLOWORLD_H
```

```cpp
// ../Utils/HelloWorld.cpp:
#include "llvm/Transforms/Utils/HelloWorld.h"

using namespace llvm;

PreservedAnalyses HelloWorldPass::run(Function &F,
                                      FunctionAnalysisManager &AM) {
  errs() << F.getName() << "\n";
  return PreservedAnalyses::all();
}
```

### register pass

在 `llvm/lib/Passes/PassRegistry.def` 添加: `FUNCTION_PASS("helloworld", HelloWorldPass())`

在 `llvm/lib/Passes/PassBuilder.cpp` 添加: `#include "llvm/Transforms/Utils/Helloworld.h"`

### run our pass with opt

```shell 
cmake --build build/ --target opt

build/bin/opt -disable-output /tmp/a.ll -passes=helloworld
```

## register pass as plugins

在 llvm-project 目录创建一个 CMake 项目, CMakeLists.txt 如下:

```cmake
add_llvm_pass_plugin(MyPass, source.cpp)
```

此 Pass 必须向新 PassManager 提供下列两个中至少一个入口:
1. 用于静态注册的入口点, 需要设置 `LLVM_${NAME}_LINK_INTO_TOOLS=ON`, 集成入 LLVM
```cpp
llvm:PassPluginLibraryInfo get##Name##PluginInfo();
```
2. 用于动态加载的入口点, 例子详见 `llvm/examples/Bye`
```cpp
extern "C" ::llvm::PassPluginLibraryInfo llvmGetPassPluginInfo() LLVM_ATTRIBUTE_WEAK;
```

> 详见 https://llvm.org/docs/WritingAnLLVMNewPMPass.html