#Requires -Version 7.4
<#
后台同步图片，前台提交并推送笔记，最后显示同步输出

新设备优先运行：
    rclone bisync ali-oss:yay-waves/til/ ./assets 
        --resync --resync-mode path1 --compare size,modtime --fast-list
#>

$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
$assets = Get-Item -LiteralPath "$repo/assets"
if (!$assets.PSIsContainer -or ($assets.Attributes -band [IO.FileAttributes]::ReparsePoint)) {
    throw 'assets must be an ordinary local directory.'
}
$sync = Start-Job -ArgumentList $repo, $assets.FullName -ScriptBlock {
    param($repo, $assets)
    rclone bisync 'ali-oss:yay-waves/til/' $assets --compare size,modtime --fast-list --verbose
    if ($LASTEXITCODE) { throw 'rclone bisync failed.' }
}
$gitError = $null
try {
    git -C $repo add -A
    if ($LASTEXITCODE) { throw 'git add failed.' }
    git -C $repo diff --cached --quiet
    if ($LASTEXITCODE -eq 1) {
        git -C $repo commit -m (Get-Date -Format 'yyMMdd')
        if ($LASTEXITCODE) { throw 'git commit failed.' }
    } elseif ($LASTEXITCODE) { throw 'git diff failed.' }
    git -C $repo push
    if ($LASTEXITCODE) { throw 'git push failed.' }
} catch { $gitError = $_ }
$sync | Receive-Job -Wait -ErrorAction Continue
$syncFailed = $sync.State -ne 'Completed'
$sync | Remove-Job -Force
if ($gitError) { throw $gitError }
if ($syncFailed) { throw 'rclone bisync failed.' }
