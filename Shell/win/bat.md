`.bat` 批处理命令和 CMD 命令语法不太相同. 和 Bash 更不同.

```bat
@echo off
cd $Dir
File.exe arg
pause
```

> 注意中文要转为 ANSI 编码. 不区分大小写.   
> CMD 使用 `%var%` 引用变量值, BAT 使用 `%%var%%` 

### `%`

1. `%var%` 变量变量
2. `%[0-9]` 对形式参数的引用

### `^`

转移字符, 常用来换行 `end of line^ new line`

### `*`

通配符:
- `?` 一个字符
- `*` 多个字符


## `for`


遍历文件: 打印文件和目录名

```cmd
for %i in (f:\env\*) do echo %i
```

遍历文件夹: 仅匹配目录名, 不匹配文件名. 

```cmd
for /d %i in (f:\env\*,f:\env\rust\*) do echo %i
```

遍历目录树: 递归执行循环, 可遍历至最深层.

```cmd
for /r f:\env %i in (*) do echo %i
```

遍历数字列表: 依次打印数字 `1, 2, 3, 4, 5`

```cmd
for /l %i in (1, 1, 5) do echo %i
```

### 获取命令返回值

在linux下得到命令结果非常简单, 如 `var = $(scripts)`

但 cmd 比较弱, 也没有变量概念, 这时候常使用`for /f`针对命令执行for, 来达到相同效果. 比如:

```bash
# 使用for /f 获取当前目录路径
for /f %i in ('chdir | findstr path') do (echo %i)
```

### 删除子目录文件

下面是一个匹配并删除二级子目录下对应目录的例子.

```
for /d %i in (D:\yjw\repo\*) do (for /d %d in (%i\*) do (del %d\translations && rmdir %d\translations))
```

> ==你敢相信这是21世纪的命令行命令?==