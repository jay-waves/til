反汇编框架, 集成了很多工具. 定位大概是 capstone 之上, ghidra 之下. 和 LLVM 类似, Radare2 分为 反汇编/反编译/调试 等步骤, 信息分为 汇编指令/基本块/函数/模块 几个层次, 并引入中间语言 ESIL 和虚拟机. 

Radare2 主体是命令行程序, r2pipe 通过管道提供了对不同语言的功能绑定, r2papi 在 r2pipe 之上构建了更丰富的接口 (并非丰富). 这些 api 都是调用命令行, 然后读取 JSON 输出. 推荐使用动态脚本语言, 解析 JSON 更方便.

`a`: 数据流, 数据引用, 符号分析, 控制流和调用流分析.
- `abb` 分析基本块
- `af` 分析函数
- `ai` 分析地址 (perms, stack, heap...)
- `ax` 分析引用

`af`, 函数分析
- 地址, 基本块, 名称, 参数, 栈帧大小 (`afl`)
- 向函数中插入新基本块
- 函数调用树

`afv`, 函数局部变量分析
- `afvR [varname]` 局部变量被读取的指令地址 (READ)
- `afvW [varname]` 局部变量被写入的指令地址 (WRITE)
- `afva` 分析所有参数和局部变量
- `afta` 分析所有参数和局部变量的类型

`t`, 按类型分析
- `ts`, 所有 `struct`
- `te`, 所有 `enum`
- `tt`, 所有 `typedef`
- `tpv` show offset formatted for given type
