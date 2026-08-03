Powershell7 比较稳定的配置位置，应该在 `~/Documents/Powershell/Microsoft.PowerShell_profile.ps1` 

编辑方式是： `nvim $PROFILE`

```powershell
# using utf-8
$OutputEncoding = [System.Text.Encoding]::UTF8
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

# diable auto-update detect 
[Environment]::SetEnvironmentVariable( 'POWERSHELL_UPDATECHECK', 'Off', 'User')

# prompt
function prompt {

    $ok = $?
    $exitCode = $LASTEXITCODE
    $time = Get-Date -Format "HH:mm:ss"

    $checkIcon = [char]::ConvertFromUtf32(0xe63f)
    $errorIcon = [char]::ConvertFromUtf32(0xf071)

	# 仅探测当前 Git 分支
    $gitBranch = git symbolic-ref --quiet --short HEAD 2>$null
    $gitResult = $LASTEXITCODE

    Write-Host ""
    Write-Host ("─" * 80) -ForegroundColor DarkGray

    Write-Host " $time" -ForegroundColor DarkGray -NoNewline

    if ($ok) {
        Write-Host " $checkIcon" -ForegroundColor Green -NoNewline
    } else {
        Write-Host " $errorIcon($exitCode)" -ForegroundColor Red -NoNewline
    }

    Write-Host ""

    Write-Host " $(Get-Location)" -ForegroundColor Blue -NoNewline

	if ($gitResult -eq 0) {
        Write-Host "   $gitBranch" -ForegroundColor DarkYellow -NoNewline
    }

    return "`n ❯❯ "
}

# zoxide:
Invoke-Expression (& { (zoxide init powershell | Out-String) })

# 如果用 uutils，请禁用 Powershell 内置的 Alias
Remove-Alias ls,cat,cp,mv,rm,curl,wget,sort,tee,where,clear,man -Force -ErrorAction SilentlyContinue
```

## 开发环境*

```powershell
Powershell Development Environment for VS2022
$vsDevShellModule = "C:\Program Files\Microsoft Visual Studio\18\Insiders\Common7\Tools\Microsoft.VisualStudio.DevShell.dll"
Import-Module -Name $vsDevShellModule -ErrorAction Stop

Enter-VsDevShell -VsInstanceId 69a2bc64 `
                -SkipAutomaticLocation `
                -DevCmdArguments "-arch=x64 -host_arch=x64"

Import-Module 'C:\Development\C\vcpkg\scripts\posh-vcpkg'
```

## 导入自定义脚本

比如各类 cli 工具的补全脚本：

```powershell
$completionDir = Join-Path (Split-Path -Parent $PROFILE) 'completions'
Get-ChildItem -Path $completionDir -Filter *.ps1 | ForEach-Object {
    . $_.FullName
}
```

## 三方模块

### 命令补全模块

(powershell7) 实时抽屉模式：

```powershell
Install-Module -Name PSReadLine -AllowPrerelease -Scope CurrentUser -Force -SkipPublisherCheck
Set-PSReadLineOption -ShowToolTips
Set-PSReadLineOption -PredictionViewStyle ListView
```

行内模式，搭配 Fzf 模块：

```powershell
# PSReadLine
Set-PSReadLineOption -PredictionSource History
Set-PSReadLineOption -PredictionViewStyle InlineView
# lazy load PSFzf on Ctrl+R
Set-PSReadLineKeyHandler -Chord 'Ctrl+r' -ScriptBlock {
    if (-not (Get-Module PSFzf)) {
        Import-Module PSFzf -ErrorAction Stop
    }

    Invoke-FzfPsReadlineHandlerHistory
}

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

### posh-git 

建议不要用 posh-git，会拖慢 200ms 启动速度。自己在 prompt 里加入 `git symbolic-ref` 探测分支就够了。

```powershell 
Import-Module posh-git
```
