## Windows PE/COFF

Win32 平台的标准**可执行文件格式**为 PE (Portable Executable), 也源于 COFF 格式. 目标是在不同版本/硬件的 Windows 平台上使用. 事实上, Visual C++ 编译器产生的**目标文件格式**仍是 COFF 格式, 因此统称为 Windows PE/COFF.

### COFF

因为 PE 文件装载时, 直接映射到进程的虚拟空间中运行, 所以 PE 可执行文件有时也叫做**映像文件.**

| 名称            | 字段                     | 描述 |
| --------------- | ------------------------ | ---- |
| 映像头 (文件头) | `IAMGE_FILE_HEADER`      | `VC\PlatformSDK\include\WinNT.h`     |
| 段表            | `IMAGE_SECTION_HEADER[]` |      |
| 代码段                | `text`                   |      |
|  数据段               | `data`                   |      |
|                 | `.drectve`               |      |
|                 | `debug$S`                |      |
|                 | ...                      |      |
| 符号表                |                          |      |


COFF 文件头:
```sh
> dumpbin hello.obj

File Type: COFF OBJECT

FILE HEADER VALUES
            8664 machine (x64)
               E number of sections
        66947BE2 time date stamp Mon Jul 15 09:31:14 2024
             4D6 file pointer to symbol table
              30 number of symbols
               0 size of optional header
               0 characteristics


```

段表信息, 一个结构体代表一个段 (类似 `Elf32_Shdr` ):

```c
// in WinNT.h
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