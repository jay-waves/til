DWARF (Debugging With Attributed Record Foramt):
- 编译单元 (Complie Unit)
- 类型信息 (Type Information)
- 变量声明 (Variable Declarations)
- 源码行号 (Line Number Information)
- 宏定义 (Macro Definitions)

libdwarf, dwarfdump

DWARF 符号表 (Debugging With Attributed Record Formats):
- 唯一ID `ubyId`
- 符号名称 `szName`
- 符号类型标签 `ubyTag`, 属于枚举类型 `DwarfTag`:
	- `DW_TAG_variable`: 变量
	- `DW_TAG_subprogram`: 函数
	- `DW_TAG_structure_type`: 结构体
	- `DW_TAG_array_type`: 数组
- 层级 `byLevel`
- 变量符号类型 `dwType`, 如 `int, char, struct`
- 基础类型 `dwBaseType`, 指基本数据类型 (DataTypeX)
	- `dtx_int32`
	- `dtx_float`
	- `dtx_struct`
- 符号长度 `udwLength`, 所占字节长度
- 内存类型 `udeMemoryType`
	- `0x80000` ROM
	- `0x80001` iRAM
	- `0x80002` xRAM
	- `0x40000` 寄存器区
- 基地址 `udwBaseAddr`
- 偏移量 `dwOffset`, 表示符号在区域或结构体内的偏移
- 位大小 `byBitSize`, 位偏移 `byBitOffset`