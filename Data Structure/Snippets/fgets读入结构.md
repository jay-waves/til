## 文件读入
- 注意: `fgets()`不会自动处理`\n`
```c
#define  SIZE
    FILE *fp_in;
    fp_in = fopen("file.path", "r");
    while (fgets(str, SIZE, fp_in) != NULL)
    {
        len = strlen(str);
        str[len - 1] = 0;//将\n置为0
        Insert(str, t);
    }
```