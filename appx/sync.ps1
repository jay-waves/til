#Requires -Version 7.4
<#
依次提交、推送笔记并同步图片

新设备优先运行：
    rclone bisync ali-oss:yay-waves/til/ ./assets 
        --workdir .oss-assets/bisync 
        --resync --resync-mode path1 --compare size,modtime --fast-list
#>

$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
$assets = Get-Item -LiteralPath "$repo/assets"
if (!$assets.PSIsContainer -or ($assets.Attributes -band [IO.FileAttributes]::ReparsePoint)) {
    throw 'assets must be an ordinary local directory.'
}
git -C $repo add -A
if ($LASTEXITCODE) { throw 'git add failed.' }
git -C $repo diff --cached --quiet
if ($LASTEXITCODE -eq 1) {
    git -C $repo commit -m (Get-Date -Format 'yyMMdd')
    if ($LASTEXITCODE) { throw 'git commit failed.' }
} elseif ($LASTEXITCODE) { throw 'git diff failed.' }
git -C $repo push
if ($LASTEXITCODE) { throw 'git push failed.' }
rclone bisync 'ali-oss:yay-waves/til/' $assets.FullName --workdir "$repo/.oss-assets/bisync" --compare size,modtime --fast-list --verbose
if ($LASTEXITCODE) { throw 'rclone bisync failed.' }
