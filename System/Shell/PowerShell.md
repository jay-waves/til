
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

## Invoke-WebRequest

Powershell 中 `curl` 实际是 `Invoke-WebRequest` 的别名 (不是[原来的工具](网络调试.md#`curl`)), **参数**有变化 (真的坑):

- `-Uri` 
- `-Proxy` --> `-x`
- `-Headers` --> `-H`
- `-ContentType` --> `-X`

## 变量

pwsl 其实不区分大小写, 别被命令吓到了 (🖤)

```powershell
$MyVar = "hello, world!"

Write-Output $MyVar

Get-Type $MyVar

$homeDir = $env:USERPROFILE
```

## 遍历

```powershell
Get-ChildItem -Filter *.png | ForEach-Object {
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

## 命令补全模块

(powershell7)

```powershell
Install-Module -Name PSReadLine -AllowPrerelease -Scope CurrentUser -Force -SkipPublisherCheck
Set-PSReadLineOption -ShowToolTips
Set-PSReadLineOption -PredictionViewStyle ListView
```