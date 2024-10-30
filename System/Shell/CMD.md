寻求帮助: 
1. `/?`
2. `help for cmd`
3. **如果没有现成命令, 不要试图用 CMD 解决问题**
4. **推荐使用 PowerShell**

### 复制文件及文件夹

复制文件及文件夹

```cmd
xcopy source\path target\path /sfh 
```

- `/s` 复制目录和子目录 (非空)
- `/f` 显示完整的来源和文件名
- `/h` 复制隐藏文件

### 删除文件及文件夹

```cmd
rmdir /s /p path\to\dir
/s 递归删除
/p 静默, 不要二次确认

del \path\to\file
```

### 使用 utf-8 编码

`hcp 65001`
