## a.out

Ken Thompson 创建原始 Unix 系统时, 使用的可执行文件格式为 `a.out`, 是 "assembler 
output" 的缩写. 编译器默认输出文件名也叫 `a.out`. 文件结构相对简单, 难以实现动态
链接, 已被淘汰.

## ELF

现代可执行文件格式包括 windows 下的 PE (Portable Executable) 和 Linux 下的 ELF 
(Executable and Linkable Foramt), 它们都是 COFF (Common File Format) 格式的变种.

目标文件, 动态链接库以及静态链接库*都*是可执行文件格式.

| COFF 文件类型                          | 描述                                   | linux 实例                                | windows 实例 |
| -------------------------------------- | -------------------------------------- | ----------------------------------------- | ------------ |
| 可重定位文件 <br> (Relocatable File)   | 包括尚未链接的目标文件 (object file), 以及静态链接库 (staic linking library)| `.o, .a` |     `.obj, .lib`         |
| 可执行文件 <br> (Executable File)      | 可直接执行程序                         | /bin/bash    |    `.exe`           |
| 共享目标文件 <br> (Shared Object File) |  动态链接库 (DLL, dynamic linking library)                                      |`.so`        |   `.dll`           |
| 核心转储文件 <br> (Core Dump File)     | 进程意外终止时, 系统将进程执行信息输出 | linux core dump                           |              |

```bash
> file top
top: ELF 64-bit LSB pie executable, x86-64, version 1 (SYSV), dynamically linked, interpreter /lib64/ld-linux-x86-64.so.2, BuildID[sha1]=9b122594b23a8d7c0451ae12ac144af377a16588, for GNU/Linux 3.2.0, stripped

> file libceres.so.2.0.0
libceres.so.2.0.0: ELF 64-bit LSB shared object, x86-64, version 1 (SYSV), dynamically linked, BuildID[sha1]=e8b64dab49f67ec0e24fa8c03961463f76892c19, stripped
```

## 目标文件内容

以源码[^1]为例, 说明链接过程 ELF 文件内容的变化. 可参考 [Linux 内存空间分布](../../System/Memory/Linux%20内存空间分布.md) 查看.


```c
int printf( const char* format, ...);

int global_init_var = 84;
int global_uninit_var;

void func1(int i)
{
  printf( "%d\n", i);
}

int main(void)
{
  static int static_var = 85;
  static int static_uninit_var;
  int var = 1;
  int uninit_var;
  func1( static_var + static_uninit_var + var + uninit_var );
  return var;
}
```

查看 elf 文件格式的工具有 `objdump -h`, `readelf`, `llvm-objdump -h`

```bash
> gcc -c test_elf.c # compile without linking
> llvm-objdump test_elf.o -h

test_elf.o:     file format elf64-x86-64

Sections:
Idx Name               Size     VMA              Type
  0                    00000000 0000000000000000
  1 .text              00000062 0000000000000000 TEXT
  2 .rela.text         00000078 0000000000000000
  3 .data              00000008 0000000000000000 DATA
  4 .bss               00000008 0000000000000000 BSS
  5 .rodata            00000004 0000000000000000 DATA
  6 .comment           0000002c 0000000000000000
  7 .note.GNU-stack    00000000 0000000000000000
  8 .note.gnu.property 00000020 0000000000000000
  9 .eh_frame          00000058 0000000000000000 DATA
 10 .rela.eh_frame     00000030 0000000000000000
 11 .symtab            00000138 0000000000000000
 12 .strtab            00000061 0000000000000000
 13 .shstrtab          00000074 0000000000000000
```

常见 ELF 通常包括如下节 (Section):

|                 | 名称                          | 信息                                                                                 | sh_type        | sh_flag                    |
| --------------- | ----------------------------- | ------------------------------------------------------------------------------------ | -------------- | -------------------------- |
| ELF Header      |                               | 入口地址, 目标平台, ELF类型, 段表                                                    |                |                            |
| .text           | 代码段                        | 可执行代码                                                                           | `SHT_PROGBITS` | `SHF_ALLOC, SHF_EXECINSTR` |
| .rela.text      | 重定位表 (relocation table)   | 记录代码和数据段中需要地址重定位的位置                                               | `SHT_REL`      |                            |
| .data           | 数据段                        | 记录初始化的全局变量或局部静态变量                                                   | `SHT_PROGBITS` | `SHF_ALLOC, SHF_WRITE`     |
| .bss            | BSS段                         | 记录未初始化的全局变量或局部静态变量. 由于默认值为 0, 所以实际没有内容.              | `SHT_NOBITS`   | `SHF_ALLOC, SHF_WRITE`     |
| .rodata         | 只读数据段                    | 存放字符串常量, 全局常量变量.                                                        | `SHT_PROGBITS` | `SHF_ALLOC`                |
| .comment        | 注释信息段                    | 编译器版本信息等, 如 `GCC:(Ubuntu-22.04)11.4.0`/ `.note` 字段还会存储更多编译器信息. | `SHT_PROGBITS` |                            |
| .note.GNU-stack | 堆栈提示段                    |                                                                                      | `SHT_NOTE`     |                            |
| .line           | 调试时的行号表                | 源代码行号与编译后指令的对应表                                                       | `SHT+PROGBITS` |                            |
| .debug          | 调试信息                      |                                                                                      | `SHT_PROGBITS` |                            |
| .strtab         | 字符串表                      |                                                                                      | `SHT_STRTAB`   |                            |
| .symtab         | 符号表                        | 详见 [编译/链接/符号](符号.md)                                                       | `SHT_SYMTAB`   |                            |
| .shstrtab       | 段名表 (Section String Table) |                                                                                      | `SHT_STRTAB`   | (`SHF_ALLOC`)              |

[动态链接](动态链接.md)的共享目标文件还包括以下几个节:

|          | 名称             | 信息                                 | sh_type       | sh_flag                    |
| -------- | ---------------- | ------------------------------------ | ------------- | -------------------------- |
| .hash    | 符号哈希表       | 用于加速动态链接时查找符号的过程     | `SHT_HASH`    | `SHF_ALLOC`                |
| .dynstr  | 动态符号字符串表 | 动态链接时, 帮助动态符号表存储字符串 |               |                            |
| .dynamic | 动态链接信息     |                                      | `SHT_DYNAMIC` | `SHF_ALLOC`, (`SHF_WRITE`) |
| .dynsym  | 动态符号表       | 存储动态链接相关的符号, `.symtab` 中也存储这些符号                                      |               |                            |

### 文件头

文件头部的 `Magic` 字段表示了 Linux 可执行文件类型, 如 ELF 文件魔数的前四个字节
为 `0x7f e l f`, 而 Java 可执行文件的魔数为 `c a f e`, 脚本语言可执行文件 (如 
Python 和 Perl) 的魔数为 `# !`, 后面跟字符串标注具体类型, 如 `#!/usr/bin/perl`

```bash
> readelf test.o --all
ELF Header:
  Magic:   7f 45 4c 46 02 01 01 00 00 00 00 00 00 00 00 00
  Class:                             ELF64
  Data:                              2's complement, little endian
  Version:                           1 (current)
  OS/ABI:                            UNIX - System V
  ABI Version:                       0
  Type:                              REL (Relocatable file)
  Machine:                           Advanced Micro Devices X86-64
  Version:                           0x1
  Entry point address:               0x0
  Start of program headers:          0 (bytes into file)
  Start of section headers:          1048 (bytes into file)
  Flags:                             0x0
  Size of this header:               64 (bytes)
  Size of program headers:           0 (bytes)
  Number of program headers:         0
  Size of section headers:           64 (bytes)
  Number of section headers:         14
  Section header string table index: 13
```

```c
// in /usr/include/elf.h

typedef sturct {
	/*
	 * Maigc: 0x7f 0x45 0x4c 0x46 ( \d ELF )
	 * Class: ELF32
	 * Data: 2's complement, little endian.
	 * Version: 1
	 * OS/ABI: Unix - System V
	 * ABI Version: 0
	 */
	unsigned char e_ident[16];
	/*
	 * Type: REL (Relocatable file). 
	 * ET_REL=1 for relocatable file, 
	 * ET_EXEC=2 for executable file, 
	 * ET_DYN=3 for shared object file.
	 */
	Elf32_Half e_type;
	/*
	 * Machine: Intel 80386, target platform.
	 * EM_M32=1 for AT&T WE 32100,
	 * EM_386=3 for Intel x86,
	 * EM_860=6 for Intel 80860.
	 */
	Elf32_Half e_machine;
	Elf32_Word e_version;
	/* 
	 * address in 32x, uint32_t.
	 * Entry point address: 0x0
	 */
	Elf32_Addr e_entry; // uint32_t, address in 32x
	/*
	 * start of headers: 0 (bytes into file), uint16_t offset
	 */
	Elf32_Off e_phoff;  
	/*
	 * start of section headers: 280 (bytes into file).
	 * offset of section table.
	 */
	Elf32_Off e_shoff;
	...
	/*
	 * Sectioon header stirng table index. 
	 * e.t. index of .shstrtab
	 */
	Elf32_Off e_shstrndx;
	
} Elf32_Ehdr;

... Elf64_Ehdr;
```

### 段表

Section Header Table 保存了 ELF 文件的各个节信息, 包括: 节名, 节长度, 在文件中的偏移, 类型, 读写权限等. 节表在 ELF 文件中的位置由 `e_shoff` 记录.

节表结构是 `Elf32_Shdr` 结构体的数组. 

```c
// section descriptor in /usr/include/elf.h
typedef struct {
	Elf32_Word sh_name;
	/*
	 * SHT_NULL, 
	 * SHT_PROGBITS (program data on disk), 
	 * SHT_NOBITS (no disk occupied)
	 * SHT_SYMTAB (symbol table), 
	 * SHT_REL (relocation)
	 */
	Elf32_Word sh_type;
	/* 
	 * SHF_WRITE, writable
	 * SHF_ALLOC, memory allocation needed, like .text, .data, .bss
	 * SHF_EXECINSTR, executable, like .text
	 */
	Elf32_Word sh_flags;
	Elf32_Addr sh_addr;
	Elf32_Off  sh_offset;
	Elf32_Word sh_size;
	...
	// seciton address alignment, sh_addr % (2**sh_addralign)==0
	Elf32_Word sh_addralign;
	Elf32_Word sh_entsize;   // section entry size
} Elf32_Shdr;
```


### 代码段

反汇编代码段 `.text`

```bash
❯ llvm-objdump -d test_elf.o

test_elf.o:     file format elf64-x86-64

Disassembly of section .text:

0000000000000000 <func1>:
       0: f3 0f 1e fa                   endbr64
       4: 55                            pushq   %rbp
       5: 48 89 e5                      movq    %rsp, %rbp
       8: 48 83 ec 10                   subq    $16, %rsp
       c: 89 7d fc                      movl    %edi, -4(%rbp)
       f: 8b 45 fc                      movl    -4(%rbp), %eax
      12: 89 c6                         movl    %eax, %esi
      14: 48 8d 05 00 00 00 00          leaq    (%rip), %rax            # 0x1b <func1+0x1b>
      1b: 48 89 c7                      movq    %rax, %rdi
      1e: b8 00 00 00 00                movl    $0, %eax
      23: e8 00 00 00 00                callq   0x28 <func1+0x28>
      28: 90                            nop
      29: c9                            leave
      2a: c3                            retq

000000000000002b <main>:
      2b: f3 0f 1e fa                   endbr64
      2f: 55                            pushq   %rbp
      30: 48 89 e5                      movq    %rsp, %rbp
      33: 48 83 ec 10                   subq    $16, %rsp
	...
	
      60: c9                            leave
      61: c3                            retq
```

### 数据段

`printf()` 中的字符串常量 `%d\n\0` 被存储在了 `.rodata` 段中. 这里使用大端序, `0x54` 正是 `global_init_var` 的值 84. `0x55` 则是 `static_init_var` 的值 85.

```bash
> llvm-objdump -s test_elf.o
...

Contents of section .data:
 0000 54000000 55000000                    T...U...
...

Contents of section .rodata:
 0000 25640a00                             %d..
```

### BSS段

ELF 文件仅记录了 bss 段 (block started by symbol) 的变量需要的体积, 在程序载入内存时会开辟其空间, 但是存储时并没有实际占有磁盘空间.

```bash
Contents of section .bss:
<skipping contents of bss section at [0000, 0008)>
```

有的编译器会将 `static int x = 0` 这样的初始化为0的变量也优化放入 `.bss` 段, 此时可以用 gcc 的 `__attribute__((section(".data")))` 来强制变量放入某个段.

### 字符串表

`.strtab` 字符串表 (string table) 用来保存符号名称等; `.shstrtab` 段表字符串表 (section header string table) 用来保存*段表*中用到的字符串, 如段名 `sh_name`.

将字符串集中存储起来后, 对字符串的引用只需要记录一个相对字符串表起始的偏移即可, 不必再考虑其存储长度问题 (用 `\0` 标识结尾). 

### 调试信息

调试信息需要单独指定编译器参数 `-g` 来生成, 位于 `.debug` 字段. [调试信息](调试信息.md)有独立的标准 DWARF3 (Debug With Arbitrary Record Format), 包括源代码行和目标代码地址的对应, 函数和变量类型, 结构体定义. 

### 动态链接信息

详见 [动态链接](动态链接.md)

[^1]: 程序员的自我修养--链接, 装载与库. 俞甲子等. P61.