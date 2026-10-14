
# WASM

WASM 是一层代码的低级中间表示，各个语言编译为 wasm 指令后，由 JIT runtime 执行。
各个语言的编译器做编译时优化，WASM JIT 再做运行时优化，基本可以减少转译性能损失。

不过有两个问题：

内存问题： 
WASM 假设内存是单一线性增长的连续内存，因此没有可靠的内存收缩方式。可以认为，语言原生有一层内存管理，外面再套一层简单 GC

系统调用问题： 
WASM 需要经过 runtime API 转义为 host syscall。事实上，原生程序的系统调用基本都要删除，换成阉割版本的 WASI (WebAssembly System Interface)

WASM 沙盒和 Web 沙盒很像：
* 独立内存
* 不可随意访问宿主文件
* 不可随意访问本地网络
* 不可调用系统 API 
