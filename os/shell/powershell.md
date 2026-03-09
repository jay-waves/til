
CMD 寻求帮助:
1. `/?`
2. `help for cmd`
3. **如果没有现成命令, 不要试图用 CMD 解决问题**
4. **如果有现成命令, 一定有更严谨的 PowerShell 命令**

PowerShell 寻求帮助:

- 参数: `-Help`
- 命令: `Get-Help <cmd> -Detailed -Full -Examples`
- [官方文档](https://learn.microsoft.com/en-us/powershell/)

powershell 为 bash 和 cmd 用户准备了对应命令的别名.

| bash                     | pwsl                                                    | pwsl alias                  | cmd                     |
| ------------------------ | ------------------------------------------------------- | --------------------------- | ----------------------- |
| `grep`                   | `find-string -Pattern`                                  |                             | `findstr`               |
| `ls, dir`                | `get-childitem`                                         | `gci`                       | `dir`                   |
| `pwd`                    | `get-location`                                          | `gl`, `pwd`                 | `chdir`                 |
| `mkdir`                  | `new-item -Type Directory`                              | `ni -Type Directory`, `pwd` | `mkdir`                 |
| `rm`                     | `remove-item`                                           | `ri`, `rm`, `del`           | `del` or `rmdir`        |
| `mv`                     | `move-item`                                             | `mi`, `mv`, `move`          | `move`                  |
| `cp`                     | `copy-item`                                             | `ci`, `cp`                  | `copy`                  |
| `grep`                   | `select-string`                                         | `findstr`, `sls`            | `findstr`               |
| `echo`                   | `write-output`                                          | `echo`                      | `echo`                  |
| `cat`                    | `get-content`                                           | `gc`, `type`, `cat`         | `type`                  |
| `chmod`                  | `set-acl`                                               |                             |                         |
| `export VarName="value"` | `$env:VarName="value"`                                  |                             | `set VarName=value`     |
| `clear`                  | `cls`                                                   |                             | `cls`                   |
| `cd .. && pwd`           | `cd; pwd`                                               |                             | `cd .. & cd`            |
| `ln -s`                  | `new-item -ItemType SymbolicLink -Path ... -Target ...` |                             | `mklink` or `mklink /D` |
|                          | `get-clipboard`                                         | `gcb`                       | `clip.exe` (not same)   |
| `ip addr`                | `get-netipaddress`                                      | None                        | `ipconfig`              |
| `time`                   | `Measure-Command {Start-Process <program> -Wait}`       |                             |                         |
| `traceroute`             |                                                         |                             | `tracert`, `netsh`,     |
|                          |                                                         |                             | `pause`                 |
| `ps`, `processes`        | `Get-Process`                                           |                             | `tasklist`              |
| `pkill -9 <p_name>`      | `Stop-Process -Name <p_name> -Force`                    |                             | `taskkill /IM <p_name>` |
| `kill -9 <pid>`          | `Stop-Process -Id <pid> -Force`                         |                             | `taskkill /pid <pid>`   |
| `^`  (换行跳脱符)        | \`                                                      | `\`                         |                         |
| `where`                  | `get-command`                                           | `where`                     | `where`                        |

Powershell 中想要直接使用 CMD 中命令, 而不是别名, 请加上 `.exe` 后缀.

### Invoke-WebRequest

Powershell 中 `curl` 实际是 `Invoke-WebRequest` 的别名 (不是[原来的工具](../tracing/网络调试.md#`curl`)), **参数**有变化 (真的坑):

- `-Uri` 
- `-Proxy` --> `-x`
- `-Headers` --> `-H`
- `-ContentType` --> `-X`

### 变量

pwsl 其实不区分大小写, 别被命令吓到了 (🖤)

```powershell
$MyVar = "hello, world!"

Write-Output $MyVar

Get-Type $MyVar

$homeDir = $env:USERPROFILE
```

### 遍历

```powershell
Get-ChildItem -Filter *.avif | ForEach-Object {
	...
}

# 等价于:
gci -Filter *.avif | % {
	...
}
```

用 `$_` 引用子项的元数据:
- `$_.FullName` 含路径的文件名
- `$_.Name` 文件名
- `$_.BaseName` 不含扩展名的文件名
- `$_.Extension` 文件扩展名
- `$_.DirectoryName` 路径
- `$_.Length` 大小, 以字节为单位

注意, 使用 ForEach 管道时, 不能使用 `.` 或 `..`. 如果需要字符串命令替换, 请使用 `"$($_.FullName)\....."`

另外, 字符串插值 `"$Var"` 只支持简单变量. 复杂解析应使用子表达式 `$()`, 如 `"$($_.Name)xxxx"`. 

如果子表达式 `$()` 返回多个结果, 将会自动展开为数组 (而不是空格隔开的字符串). 这导致子表达式很难直接嵌入到批处理命令中, 比如 `mv $(fd xxx.*)` 很可能报错, 此时仍需要使用 `ForEach-Object` 遍历列表.

### 命令补全模块

(powershell7)

```powershell
Install-Module -Name PSReadLine -AllowPrerelease -Scope CurrentUser -Force -SkipPublisherCheck
Set-PSReadLineOption -ShowToolTips
Set-PSReadLineOption -PredictionViewStyle ListView
```

### 命令编辑模块

在外部编辑器中编辑命令行当前键入的命令: 

```powershell
Install-Module -Name PSReadLine -Force -Scope CurrentUser
Set-PSReadLineOption -EditMode Vi
$env:VISUAL = 'nvim' # 指定编辑器, 需要 nvim 在 PATH 中.

# 建立键绑定: alt+x
Set-PSReadLineKeyHandler -Chord Alt+x -Function ViEditVisually
```

### 命令历史模块

直接编辑历史:
```powershell
nvim (Get-PSReadlineOption).HistorySavePath
```

## Q&A 1

管道和重定向编码问题: Powershell7 默认支持 [UTF-8](../sss/字符编码.md), 但是不同平台上的一些功能又依赖于平台本身的语言设置. 在 Windows 上, 默认中文编码使用了非 UTf-8 编码, 导致使用管道出现乱码.

<https://github.com/PowerShell/PowerShell/issues/17523> 解决办法如下:

```powershell
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
```

<https://stackoverflow.com/questions/40098771/changing-powershells-default-output-encoding-to-utf-8>

## Q&A 2

执行 `.ps1` 脚本遇到策略问题, 使用管理员模式执行:

```powershell
# 允许本地脚本, 禁止远程脚本
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope LocalMachine

# 或, 完全绕过验证
Set-ExecutionPolicy -ExecutionPolicy ByPass -Scope LocalMachine
```