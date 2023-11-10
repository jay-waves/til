- 名称: malloc_structure
- 实现:

```c
*ptr = (*)malloc( * sizeof());
if (ptr == NULL)
{
  printf("Memory allocation failed!\n");
  exit(0);
}

//place somewhere
free(ptr);
```