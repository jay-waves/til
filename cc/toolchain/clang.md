
llvm 是编译器框架, clang 作为前端, 则可以*控制和驱动*整个编译流程 (调用 llvm 库各种工具, 实现编译). 使用 clang 直接驱动编译过程时, 所有 `IR` 都是内存格式. 在编译器中作为前端. 

btw, gcc 也是驱动 cc, as, ld 程序来编译的.

### 编译动态库

`-fPIC`: 生成"位置无关代码" (Position-Independent Code), 即代码可以被加载到任何内存位置上运行, 而不是在某一固定的内存地址. 有利于程序动态链接.

`-shared` 指示编译器生成动态链接库 (如 `.so`)

### llvm-confg

快捷获取使用 llvm 相关工具时所需要的编译参数.

```shell
clang++ -shared -fPIC -o hello.so hello.cpp \
	$(llvm-config --cxxflags) \
	$(llvm-config --ldflags) \
	$(llvm-config --libs) \
	$(llvm-config --system-libs)
```

### llvm-project 

clang 实际调用的 LLVM 工具链：
- bugpoint: 用于定位出错的 Pass
- llvm-ar: archiver
- llvm-as: `.ll` -> `.bc`
- llvm-dis: `.bc` -> `.ll`
- llvm-link: link ir
- lli: llvm JIT  解释器, 直接执行 bitcode
- llc: `.bc` -> `.s`
- opt: IR 优化器/分析器
- llvm-objdump: `.o` -> `.ll` 
- lldb : 内置调试器

