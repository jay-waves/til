Powershell7 比较稳定的配置位置，应该在 `~/Documents/Powershell/Microsoft.PowerShell_profile.ps1` 

编辑方式是： `nvim $PROFILE`

```powershell
# using utf-8
$OutputEncoding = [System.Text.Encoding]::UTF8
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

Set-PSReadLineOption -ShowToolTips
Set-PSReadLineOption -PredictionViewStyle ListView

# set command editor
Set-PSReadLineOption -EditMode Vi
$env:VISUAL = 'nvim' 
$env:EDITOR = 'nvim'
# using alt+x  
Set-PSReadLineKeyHandler -Chord Alt+e -Function ViEditVisually

# prompt
function prompt {
    $ok = $?
    $exitCode = $LASTEXITCODE
    $time = Get-Date -Format "HH:mm:ss"

    $checkIcon = [char]::ConvertFromUtf32(0xe63f)
    $errorIcon = [char]::ConvertFromUtf32(0xf071)

    # ===== 分隔线 =====
    Write-Host ""
    Write-Host ("─" * 80) -ForegroundColor DarkGray

    # ===== 时间 + 状态 =====
    Write-Host " $time" -ForegroundColor DarkGray -NoNewline

    if ($ok) {
        Write-Host " $checkIcon" -ForegroundColor Green -NoNewline
    } else {
        Write-Host " $errorIcon($exitCode)" -ForegroundColor Red -NoNewline
    }

    Write-Host ""

    # ===== 路径 =====
    Write-Host " $(Get-Location)" -ForegroundColor Blue -NoNewline

    # ===== 输入区 =====
    return "`n ❯❯ "
}

# zoxide:
Invoke-Expression (& { (zoxide init powershell | Out-String) })

# posh-git
Import-Module posh-git

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

```powershell
$completionDir = Join-Path (Split-Path -Parent $PROFILE) 'completions'
Get-ChildItem -Path $completionDir -Filter *.ps1 | ForEach-Object {
    . $_.FullName
}
```