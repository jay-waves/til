寻求帮助: 
1. `/?`
2. `help for cmd`
3. **如果没有现成命令, 不要试图用 CMD 解决问题**
4. **推荐使用 PowerShell**

### `xcopy`

复制文件及文件夹

```cmd
xcopy source\path target\path /sfh 
```

- `/s` 复制目录和子目录 (非空)
- `/f` 显示完整的来源和文件名
- `/h` 复制隐藏文件

### `dir`

- `/ad` 显示目录
- `/ah` 显示隐藏文件
- `/as` 显示系统文件
- `/b` 只显示文件名
- `/s` 显示目录和所有子目录的文件
- `/t[:caw]` 时间

## 文件操作

```cmd
rmdir /s /p path\to\dir
/s 递归删除
/p 静默, 不要二次确认

del \path\to\file

type 

/?
```

## 使用 utf-8 编码

`hcp 65001`
