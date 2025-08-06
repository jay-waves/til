## Windows PE/COFF

Win32 平台的标准**可执行文件格式**为 PE (Portable Executable), 也源于 COFF 格式. 目标是在不同版本/硬件的 Windows 平台上使用. 事实上, Visual C++ 编译器产生的**目标文件格式**仍是 COFF 格式, 可执行文件才是 PE 格式, 因此统称为 Windows PE/COFF.

## COFF

因为 PE 文件装载时, 直接映射到进程的虚拟空间中运行, 所以 PE 可执行文件有时也叫做**映像文件.**

| 名称            | 字段                     | 描述 |
| --------------- | ------------------------ | ---- |
| 映像头 (文件头) | `IAMGE_FILE_HEADER`      | `VC\PlatformSDK\include\WinNT.h`     |
| 段表            | `IMAGE_SECTION_HEADER[]` |      |
| 代码段                | `.text`                   |      |
|  数据段               | `.data`                   |      |
|                 | `.drectve`               |      |
|                 | `.debug$S`                |      |
|                 | ...                      |      |
| 符号表                |                          |      |


### COFF 文件头
```sh
> dumpbin hello.obj

File Type: COFF OBJECT

FILE HEADER VALUES
           0x14C machine (x64)
               5 number of sections
        66947BE2 time date stamp Mon Jul 15 09:31:14 2024
             4D6 file pointer to symbol table
              30 number of symbols
               0 size of optional header
               0 characteristics


```

段表信息, 一个结构体代表一个段 (类似 `Elf32_Shdr` ) 如下. `MACHINE_` 通常是 x86 (`0x14c`), 虽然也支持 `ARM, MIPS`.

```c
// in VC\PlatformSDK\include\WinNT.h
#define IMAGE_FILE_MACHINE_ 0x14c

typedef struct _IMAGE_FILE_HEADER {
	WORD Machine;
	WORD NumberOfSections;
	DWORD TimeDateStamp;         // PE 文件创建时间
	DWORD PointerToSymbolTable;  // 符号表位置
	DWORD NumberOfSymbols;
	WORD SizeOfOptionalHeader;   // 仅 PE 格式有 Optioanl Header
	WORD Characteristics;
} IMAGE_FILE_HEADER, *PIMAGE_FILE_HEADER;

typedef struct _IMAGE_SECTION_HEADER {
	BYTE Name[8];
	unioin {
		DWORD PhysicalAddress;
		DWORD VirtualSize; /* 该段被加载至内存后的大小 */
	} Misc;
	DWORD VirtualAddress; /* 该段被加载至内存后的虚拟地址 */
	 /* 该段文件中大小, 如 .bss 段 的VirtualSize != SizeofRawData */
	DWORD SizeofRawData; 
	DWORD PointerToRawData;
	DWORD PointerToRelocations;
	DWORD PointerToLinenumbers;
	WORD NumberOfRelocations;
	WORD NumberOfLinenumbers;
	DWORD Characteristices; /* 标志位: 段类型, 对齐方式, 读写权限 */
} IMAGE_SECTION_HEADER, *PIAMGE_SECTION_HEADER;
```

### Directve 

`.drectve` 是传递给链接器的指令. 该段是信息段, 并非程序数据, 在链接时被抛弃. 链接指令详见 [msvc/link](../工具链/msvc.md).

## PE 

和 COFF 的区别:
- 文件开头是 DOS MZ 文件头 (DOS MZ File Header and Stub) 
- `IMAGE_FILE_HEADER` 文件头结构扩展为 `IMAGE_NT_HEADERS`, 其中包含了新增扩展头部结构 `PE Optional Header`


MZ 是古老的 DOS 操作系统的可执行文件格式. Windows 使用 MZ 可执行文件头, 是早期兼容 DOS 时的设计. 
PE 真正的开头是 `IMAGE_NT_HEADERS`, 包含了一个标记 ``

```c
typedef struct _IMAGE_NT_HEADERS {
	DWORD Signature;   /* 对于 PE 格式而言, 是小端序的 'PE\0\0' 字符. */
	IMAGE_FILE_HEADER FileHeader;
	IMAGE_OPTIONAL_HEADER OptionalHeader;
} IMAGE_NT_HEADERS;

typedef struct _IMAGE_OPTIONAL_HEADER {
	// Standard fields
	WORD Magic;
	BYTE MajorLinkerVersion;
	BYTE MinorLinkerVersion;
	DWORD SizeOfCode;
	DWORD SizeOfInitializedData;
	DWORD SizeOfUninitializedData;
	DWORD AddressOfEntryPoint;
	DWORD BaseOfCode;
	DWORD BaseOfData;
	// NT addtional fields,
	...
	IMAGE_DATA_DIRECTORY DataDirectory[...];
};

/* 
	用于快速定位和获取 PE 文件装载时需要的数据结构. 多数和动态链接相关.
	如 符号导出表, 导入表, 重定位表, 调试信息表...
*/
typedef struct _IMAGE_DATA_DIRECTORY {
	DWORD VirtualAddress;
	DWORD Size;
};
```