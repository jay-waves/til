---
revised: 2024-04-09
code: [<string.h>, <crypt.h>, <memory.h>, <wchar.h>]
---

字符串类型形如 `"whatever\0"`, 用法简单. 但要注意, 字符串长度应加上最后的 `\0` 字符, 用于标识字符串结尾.

```c
char *str1 = "HELLO";
char str2[10];
int length;

length = strlen("HELLO"); /* length = 5 */
(void) strcpy(str2,str1);
```

## string.h

所有函数原型于 `string.h` 头文件中定义.

```c
// copy one string into another
char *strcpy (char *dest, const char *src)

// compare strings with alphabetic order, 
// <0 if str1 is lexically less than str2
int strcmp(char *string1, const char *string2)
int strcasecmp(...) // case insensitvie version

// compare first n character of strings
char *strncpy(char *string1, const char *string2, size_t n)

// copy string 
size_t strlen(const char *string)

// append n characters from string2 to string1
char *strncat(char *string1, char *string2, size_t n)

// 在参数调用中, dest_string 总是在 src_string 之前
// strncat, strncmp, strncpy 可能错误操作 NULL 结束符!
```

字符串查找相关函数:
```c
// find first occurence of character c
char *strchr(const char *string, int c)

// last occurrence
char *strrchr(const char *string, int c)

// find first occureence of str2 in str1
char *strstr(const char *s1, const char *s2)

// returns a pointer to the first occurrence in str1 
// of any character from str2, or a null pointer if 
// no character from s2 exists in s1  
char *strpbrk(const char *s1, const char *s2)

// returns the number of characters at the begining of s1 that match s2.  
size_t strspn(const char *s1, const char *s2)
size_t strcspn(const char *s1, const char *s2) // not match

// tokenize
strtok...
```

## ctype.h

字符, 定义在 `ctype.h`

```c
int isalnum(int c)   // alphanumeric
int isalpha(int c)   // letter
int isascii(int c)   // ASCII
int iscntrl(int c)   // control character
int isdigit(int c)
int isgraph(int c)   // graphical character
int islower(int c)   // lowercase
int isupper(int c)
int isprint(int c)   // printable character
int ispunct(int c)   // punctuation char
int isspace(int c)
int isxdigit(int c)  // hexcadecimal digit

// conversion
int toascii(int c)
int tolower(int c)
int toupper(int c)
```

## memory.h

和字符串相关的内存操作, 定义在 `memory.h`
```c
// search for a cahr in a buffer
void *memchar (void *s, int c, size_t n)

// compare two buffer, using unsigned bytes' number.
int memcmp (void *s1, void *s2, size_t n)

// copy one buffer into another
void *memcpy (void *dest, void *src, size_t n)

// move n bytes from one buffer to another
void *memmove (void *dest, void *src, size_t n)

// set all bytes of a buffer to a given character
void *memset(void *s, int c, size_t n)
```

例子:
```c
int src[SIZE], idest[SIZE]
memcpy(dest, src, SIZE*sizeof(int));
```

## wchar.h

`wchar_t` 宽字符类型, 用于表示多字节字符集 (如 Unicode), 在 C95 标准被正式引入. 

```c
#include <wchar.h>

wchar_t str[] = L'你好, 世界'; // L 标识宽字符字面量
wprintf(L"%ls\n", str);       // 宽字符格式化输出
```

**`wchar_t` 长度不是固定的, 和平台有关**.

```c
#include <wchar.h>
#define WCHAR_SIZE (sizeof(wchar_t))
#if WCHAR_SIZE == 2
	// Windows, UTF-16
#elif WCHAR_SIZE == 4
	// Unix, UTF-32
#else
	// 
#endif
```