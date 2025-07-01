---
code: <stdlib.h>
---

## 文件打开关闭

```c
FILE *fopen(const char *path, const char *mode);

int fclose (FILE *stream);
```

| mode           | 含义                                               |
| -------------- | -------------------------------------------------- |
| `r, rb`        | 只读                                               |
| `w, wb`        | 只写. 如果文件不存在, 则创建该文件, 否则文件被截断 |
| `a, ab`        | 以追加方式                                         |
| `r+, r+b, rb+` | 以读写方式打开.                                                   |
| `w+, w+b, wb+` | 以读写方式打开. 如果文件不存在, 创建新文件, 否则文件被截断.                                                   |
| `a+, a+b, ab+`               |  以读和追加方式打开. 如果文件不存在, 则创建新文件.                                                  |

`b` 用于区分二进制文件和文本文件, 在 DOS, Windows 系统有用, Linux 不区分.

## 文件读写

支持以字符 / 字符串为单位读写文件.

```c
int fgetc(FILE* stream);
int fputc(int c, FILE *stream);
char *fgets(char *s, int n, FILE *stream);
int fputs(const char *s, FILE *stream);
int fprintf(FILE *stream, const char *format, ..); // 按某种格式读写文件
int fscanf(FILE *stream, const char *format, ...);

// 从流中读取 n 个字段, 每个字段为 size 字节, 读取结果用 ptr 指向, 返回实际读取的字段数 (尤其是读取数小于参数 num)
size_t fread(void *ptr, size_t size, size_t n, FILE *stream);
size_t fwrite(const void *ptr, size_t size, size_t n, FILE *stream);
```

## 文件定位

```c
int fgetpos(FILE *stream, fpos_t *pos);
int fsetpos(FILE *stream, const fpos_t *pos);
int fseek(FILE *stream, long offset, int whence);
```