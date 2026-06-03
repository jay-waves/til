
## Reference

程序员的自我修养--链接, 装载与库. 俞甲子, 石凡, 潘爱民. 2009.

[LLVM Language Reference](https://llvm.org/docs/LangRef.html)  

## TOC 

| toc       | en                            | zh         |
| --------- | ----------------------------- | ---------- |
| isa       | Instructions Set Architecture | 指令集架构 |
| toolchain |                               | 编译工具链 |
| linking   |                               | 链接过程   |
|           |                               |            |

## Glossary 

#### Stack

先入后出的栈数据结构, 见 [binary heap](Algorithm/内核/list.md). 也指进程内存空间的一种结构, 见 [linux-内存空间分布](../os/mem/linux-内存空间分布.md).

#### BSS

block started by symbol, ELF 文件中存储未初始化全局变量和局部静态变量的段. 见 [unix-elf](linking/unix-elf.md)

#### Heap

堆, 数据结构. 见 [binary-heap](../algo/tree/binary-heap.md). 也指进程内存空间的一种结构.

#### Static Shared Library

静态共享库.

#### Symbol Link

软链接.

#### Symbol Resolution

符号决议.

#### Bootstrap

自举

#### COFF

common object file format, ELF 格式前身. 见 [unix-elf](linking/unix-elf.md)

#### DDL

dynamic linking library, 动态链接库.

#### DSO

dynamic shared object, 动态共享对象.

#### DWARF 

Debug With Arbitrary Record Format. 通用调试记录格式. 详见 [debug-info](linking/debug-info.md)

#### ELF

executable linkable format, 可执行可连接格式. 见 [unix-elf](linking/unix-elf.md)

#### EAT

export address table, 导出地址表.

#### GOT

global offset table, 全局偏移表

### Grammar Parser

语法分析器. 见 [compilation](compilation.md)

#### Linking

链接, 程序编译的步骤. 见 [static-linking](linking/static-linking.md)

#### Name Mangling

符号改编.

#### Object File

目标文件, 编译过程中中间文件. 见 [unix-elf](linking/unix-elf.md)

#### PE

portable executable, Windows 可执行文件格式. 源于 COFF.

#### P-Code

P 码, 一种编译器中间码.

#### PIC

position-independent code, 地址无关代码. 见 [dynamic-linking](linking/dynamic-linking.md)

#### PIE

position-independent executable, 地址无关可执行文件.

#### PLT

procedure linkage table. 过程链接表. 见 [dynamic-linking](linking/dynamic-linking.md)

#### Syntax Tree

语法树, 编译前端的中间产物, 见 [compilation](compilation.md).

#### Relocation

地址重定位.

#### Semantic Analyzer

语义分析器. 见 [compilation](compilation.md).

#### Shared Library

共享库.