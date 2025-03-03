| types                                           | width                                             | description                                          |
| ----------------------------------------------- | ------------------------------------------------- | ---------------------------------------------------- |
| `short`                                         | 2B                                          |                                                      |
| `int`                                           | 4B on x32, x64                               |                                                      |
| `long`                                          | 4B on x32, x64-MSVC <br> 8B on x64-Unix |                                                      |
| `long long`                                     | 8B, after C99                                |                                                      |
| `char`                                          | 1B                                            |                                                      |
| `float`                                         | 4B                                           | IEEE754 floating-point value                         |
| `double`                                        | 8B                                           | double-precision floating-point value                |
| `size_t`                                        | `uint` on x32 <br> `ulong(long)` on x64           | store size, platform-independent.                    |
| `intptr_t, uintptr_t`                           |                                                   | store pointer                                        |
| `intmax_t, uintmax_t`                           |                                                   |                                                      |
| `int8_t, uint8_t, ...` <br> `int64_t, uint64_t` |                                                   | in `stdlib.h`, platform-independent with fixed width |
| `wchar_t`                                       | 4B on Unix (UTF-8 encoded)<br>2B on windows (UTF-16 encoded)                                                  | `stddef.h`, `wchar.h`                                                    |


https://en.wikipedia.org/wiki/C_data_types#float.h

### bool

C89 标准中没有对 `bool` 的原生支持, C99 才引入了 `stdbool.h` 头文件. 因为 `bool` 极容易和现有 C 程序的宏或定义冲突, 所以委员会使用了保留名称 `_Bool`, 将选择权交给程序员.

```c
#include <stdbool.h>

typdef bool _Bool // 实际上 _Bool 才是 c99 引入的原生布尔类型.
#define true 1
#define false 0
```

在 C99 之前, 开发者已习惯用整数, 宏或枚举模拟布尔值, 所以 `stdbool` 没有被广泛采用.

### Static variables

静态变量对函数而言是**本地的**, 只会被初始化一次 (函数第一被调用时), 每次函数调用皆访问同一对象, 直到程序结束销毁. 常用于递归.

### Struct 

```c
struct node {
	int key;
	struct node *forward[]; // zero size
}
// using like:
struct node *n = malloc(sizeof(node) + 10*sizeof(struct node*));
```

结构体按顺序从低地址到高地址排列, 但这并不是标准要求而是惯例. 结构体按其内部体积最大成员进行对齐, 结构体内部按各个成员需求进行对齐.

```c
struct data { // total size: 24, 8-byte alignment
	char   a; // offset 0,  1-byte alignment
	int    b; // offset 4,  4-byte alignment
	char   c; // offset 8,  1-byte alignment
	double d; // offset 16, 8-byte alignment
}
```

#### Bit field

位域 (bit field) 允许数据按位存储, 即并不是按 `unsigned int` 那样字节为单位存储, 编译器会尽量紧凑地排布, 允许多个位域在一个字节中连续分布. 位域节省了内存, 但是按位访问和对齐问题可能会影响性能.

```c
struct Foo {             // total size: 16 bits
	unsigned int a : 3;  // 3 bits width
	unsigned int b : 5;  // 5 bits width
	unsigned int c : 8;  // 8 bits width
};
```

由于位域的排布依赖于编译器实现, 因此在不同编译器和不同平台上, 内存分布会有不一致性.

位域常用来简化标志位 (bit flag) 的使用:

```c
// 传统方式
#define IS_A 0x01
#define IS_B 0x02
#define IS_C 0x04
unsigned int flags;

if (flags & IS_A) ...

// 使用位域
struct {
	unsigned int is_a : 1;
	unsigned int is_b : 1;
	unsigned int is_c : 1;
} flags;

if (flags.is_a) ...
```

#### Designated initialization

命名初始化 (Designated Initializtion, C99) 允许对"数组, 结构体, 联合体"以非固定元素顺序来初始化.

```c
struct Foo {
	int a;
	int b;
};

struct Foo foo = {.a = 1, .b = 2}; // 具名列表初始化

int arr[10] = { [3] = 1, [5] = 2 }; // 数组具名初始化
```

常规列表初始化:

```c
struct Foo foo {1, 2};
struct Foo foo = {0}; // a=0, b=0
```

注意, C++ 并不支持命名初始化, 但支持类似的列表初始化 (C++11):

```cpp
struct Foo {
	int a;
	int b;
	
	Foo(int a_val = 0, int b_val = 0) : a(a_val), b(b_val) {}
};

Foo foo {1, 2}; // 构造列表
Foo foo(1, 2);  // 构造函数
```

### Unions

用于表达可能有不同类型的对象. 编译器会预留 Union 中体积最大类型所占的空间.

```c
union number {
	short short_num;
	long long_num;
	double float_num;
} a_num;
```

常规用法是同时封装一个变量用于标识 Union 对象当前的类型.

```c
typedef union {
	jet jetu;
	hellicopter heli;
	cargoplane cargo;
} aircraft;

typedef struct {
	type kind; // type flag
	int speed;
	aircraft descrip;
} an_aircraft;
```

### Enumerated Types

将常量和整数绑定.

```c
// default start with index value 0, increment one by one.
// mon=0, tues=1, and so on.
enum days {mon, tues, ..., sun} week;
enum days week1, week2; // week, week1, week2 are variables
```

使用其他值:
```c
enum escapes { 
	bell='\a',
	backspace='\b',
	tab='\t',
	newline='\n'
	ret='\r'
};
```

使用其他初始值, 后续递增1.

```c
enum months {
	jan=1,
	feb, /*2*/
	mar, /*3*/
	...
	dec, /*12*/
};
```

### Type-Casting

> "Cast, if in doubt"

```c
f = (float) aint / (float) bint;
f = 10.0 / (float) cint;
```

### Funtion types

```c
typedef double func_t(double);
func_t sin, cos, sqrt;
func_t *ftpt = &sqrt;

ftpt(4);
```

### Array

#### VLA 

C99 引入的 VLA (variable length array) 允许运行时声明动态数组大小. 该内存是声明在栈上的, linux 的单线程栈上限为 `8MiB`, 超出这个值会触发 `segmentfault`, 可能造成可利用的栈溢出漏洞. 

```c
char arr[n];

```

直接在堆上声明动态数组是更明智的选择. 由于虚拟内存技术的存在, 堆可用空间可不断扩展, 上限仅受制于物理内存大小. 并且可以检测内存分配是否成功.

```c
char* arr = malloc(n * (sizeof *arr));
if (arr == NULL) 
	printf("bad allocation");
```

使用 VLA, 且仅开启 `-o1` 级别编译器优化, 比"使用固定大小数组"生成的汇编代码慢约七倍[^1], 并且会拖慢后续一系列依赖于该 VLA 的代码. 由于上述缺点, VLA 在一些编译器中甚至不被完全支持 (如 MSVC), 支持 C11 的编译器也可能不支持 C99 VLA, C++ 目前也没有支持 VLA 的计划.

[^1]: https://jorenar.com/blog/vla-pitfalls