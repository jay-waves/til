# using utf-8
$OutputEncoding = [System.Text.Encoding]::UTF8
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
# zoxide:
Invoke-Expression (& { (zoxide init powershell | Out-String) })
$env:GH_LOG_LEVEL = "info"
# $env:HTTP_PROXY = "http://127.0.0.1:7890"
# $env:HTTPS_PROXY = "https://127.0.0.1:7890"

# auto completion
Get-ChildItem -Path 'D:\bin\comple' -Filter *.ps1 | ForEach-Object {
    . $_.FullName
}
Set-PSReadLineOption -ShowToolTips
Set-PSReadLineOption -PredictionViewStyle ListView

# set command editor
Set-PSReadLineOption -EditMode Vi
$env:VISUAL = 'nvim' 
# using alt+x  
Set-PSReadLineKeyHandler -Chord Alt+x -Function ViEditVisually

# prompt
function prompt {
    $lastExitCode = $?
    $errorIcon = [char]::ConvertFromUtf32(0xf071)
    $currentTime = Get-Date -Format "MM-dd HH:mm"
    $timeIcon = [char]::ConvertFromUtf32(0xf0109)
    $branch = & git rev-parse --abbrev-ref HEAD 2>$null
    $repoInfo = $null
    if ($branch -and $branch -ne 'HEAD') {
        $remoteName = & git config --get branch.$branch.remote 2>$null
        if ($remoteName) {
            $remoteUrl = & git remote get-url $remoteName 2>$null
            if ($remoteUrl) {
                $repoName = $remoteUrl -replace '^.+github\.com[:\/](.+?)(\.git)?$', '$1'
                $repoInfo = "$branch -> $repoName"
            }
        }
    }
    $gitIcon = [char]::ConvertFromUtf32(0xf418)
    $gitInfo = if ($repoInfo) { "`n $gitIcon $repoInfo`n" } elseif ($branch) { "`n $gitIcon $branch`n" } else { "`n" }
    # output:
    Write-Host $gitInfo -ForegroundColor DarkBlue -NoNewline
    Write-Host " $timeIcon $currentTime " -NoNewline
    if (-not $lastExitCode) {
        Write-Host " $errorIcon " -ForegroundColor Red -NoNewline
    } else {
        $checkIcon = [char]::ConvertFromUtf32(0xe63f)
        Write-Host " $checkIcon " -ForegroundColor Green -NoNewline
    }
    " " + $(Get-Location) + " >> "
}

# yazi wrapper
function yy {
    $tmp = [System.IO.Path]::GetTempFileName()
    yazi $args --cwd-file="$tmp"
    $cwd = Get-Content -Path $tmp
    if (-not [String]::IsNullOrEmpty($cwd) -and $cwd -ne $PWD.Path) {
        Set-Location -LiteralPath $cwd
    }
    Remove-Item -Path $tmp
}



# =============================================================================
#
# Utility functions for zoxide.
#

# Call zoxide binary, returning the output as UTF-8.
function global:__zoxide_bin {
    $encoding = [Console]::OutputEncoding
    try {
        [Console]::OutputEncoding = [System.Text.Utf8Encoding]::new()
        $result = zoxide @args
        return $result
    } finally {
        [Console]::OutputEncoding = $encoding
    }
}

# pwd based on zoxide's format.
function global:__zoxide_pwd {
    $cwd = Get-Location
    if ($cwd.Provider.Name -eq "FileSystem") {
        $cwd.ProviderPath
    }
}

# cd + custom logic based on the value of _ZO_ECHO.
function global:__zoxide_cd($dir, $literal) {
    $dir = if ($literal) {
        Set-Location -LiteralPath $dir -Passthru -ErrorAction Stop
    } else {
        if ($dir -eq '-' -and ($PSVersionTable.PSVersion -lt 6.1)) {
            Write-Error "cd - is not supported below PowerShell 6.1. Please upgrade your version of PowerShell."
        }
        elseif ($dir -eq '+' -and ($PSVersionTable.PSVersion -lt 6.2)) {
            Write-Error "cd + is not supported below PowerShell 6.2. Please upgrade your version of PowerShell."
        }
        else {
            Set-Location -Path $dir -Passthru -ErrorAction Stop
        }
    }
}

# =============================================================================
#
# Hook configuration for zoxide.
#

# Hook to add new entries to the database.
$global:__zoxide_oldpwd = __zoxide_pwd
function global:__zoxide_hook {
    $result = __zoxide_pwd
    if ($result -ne $global:__zoxide_oldpwd) {
        if ($null -ne $result) {
            zoxide add "--" $result
        }
        $global:__zoxide_oldpwd = $result
    }
}

# Initialize hook.
$global:__zoxide_hooked = (Get-Variable __zoxide_hooked -ErrorAction SilentlyContinue -ValueOnly)
if ($global:__zoxide_hooked -ne 1) {
    $global:__zoxide_hooked = 1
    $global:__zoxide_prompt_old = $function:prompt

    function global:prompt {
        if ($null -ne $__zoxide_prompt_old) {
            & $__zoxide_prompt_old
        }
        $null = __zoxide_hook
    }
}

# =============================================================================
#
# When using zoxide with --no-cmd, alias these internal functions as desired.
#

# Jump to a directory using only keywords.
function global:__zoxide_z {
    if ($args.Length -eq 0) {
        __zoxide_cd ~ $true
    }
    elseif ($args.Length -eq 1 -and ($args[0] -eq '-' -or $args[0] -eq '+')) {
        __zoxide_cd $args[0] $false
    }
    elseif ($args.Length -eq 1 -and (Test-Path $args[0] -PathType Container)) {
        __zoxide_cd $args[0] $true
    }
    else {
        $result = __zoxide_pwd
        if ($null -ne $result) {
            $result = __zoxide_bin query --exclude $result "--" @args
        }
        else {
            $result = __zoxide_bin query "--" @args
        }
        if ($LASTEXITCODE -eq 0) {
            __zoxide_cd $result $true
        }
    }
}

# Jump to a directory using interactive search.
function global:__zoxide_zi {
    $result = __zoxide_bin query -i "--" @args
    if ($LASTEXITCODE -eq 0) {
        __zoxide_cd $result $true
    }
}

# =============================================================================
#
# Commands for zoxide. Disable these using --no-cmd.
#

Set-Alias -Name z -Value __zoxide_z -Option AllScope -Scope Global -Force
Set-Alias -Name zi -Value __zoxide_zi -Option AllScope -Scope Global -Force

Invoke-Expression (& { (zoxide init powershell | Out-String) })

# =============================================================================
