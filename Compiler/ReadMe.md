
## Reference

程序员的自我修养--链接, 装载与库. 俞甲子, 石凡, 潘爱民. 2009.

[LLVM Language Reference](https://llvm.org/docs/LangRef.html)  


## Glossary 

#### Stack

先入后出的栈数据结构, 见 [binary heap](Algorithm/内核/list.md). 也指进程内存空间的一种结构, 见 [linux 内存空间分布](System/Memory/linux%20内存空间分布.md).

#### BSS

block started by symbol, ELF 文件中存储未初始化全局变量和局部静态变量的段. 见 [Unix-ELF](Compiler/链接过程/Unix-ELF.md)

#### Heap

堆, 数据结构. 见 [binary heap](Algorithm/树/binary%20heap.md). 也指进程内存空间的一种结构.

#### Static Shared Library

静态共享库.

#### Symbol Link

软链接.

#### Symbol Resolution

符号决议.

#### Bootstrap

自举

#### COFF

common object file format, ELF 格式前身. 见 [Unix-ELF](Compiler/链接过程/Unix-ELF.md)

#### DDL

dynamic linking library, 动态链接库.

#### DSO

dynamic shared object, 动态共享对象.

#### DWARF 

Debug With Arbitrary Record Format. 通用调试记录格式. 详见 [调试信息](Compiler/链接过程/调试信息.md)

#### ELF

executable linkable format, 可执行可连接格式. 见 [Unix-ELF](Compiler/链接过程/Unix-ELF.md)

#### EAT

export address table, 导出地址表.

#### GOT

global offset table, 全局偏移表

### Grammar Parser

语法分析器. 见 [编译过程](Compiler/编译过程.md)

#### Linking

链接, 程序编译的步骤. 见 [静态链接](Compiler/链接过程/静态链接.md)

#### Name Mangling

符号改编.

#### Object File

目标文件, 编译过程中中间文件. 见 [Unix-ELF](Compiler/链接过程/Unix-ELF.md)

#### PE

portable executable, Windows 可执行文件格式. 源于 COFF.

#### P-Code

P 码, 一种编译器中间码.

#### PIC

position-independent code, 地址无关代码. 见 [动态链接](Compiler/链接过程/动态链接.md)

#### PIE

position-independent executable, 地址无关可执行文件.

#### PLT

procedure linkage table. 过程链接表. 见 [动态链接](Compiler/链接过程/动态链接.md)

#### Syntax Tree

语法树, 编译前端的中间产物, 见 [编译过程](Compiler/编译过程.md).

#### Relocation

地址重定位.

#### Semantic Analyzer

语义分析器. 见 [编译过程](Compiler/编译过程.md).

#### Shared Library

共享库.