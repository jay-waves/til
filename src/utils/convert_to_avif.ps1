Set-Location -Path (Get-Location)

$images = Get-ChildItem -File -Include "*.png", "*.jpg", "*.jpeg" -Recurse

if ($images.Count -eq 0) {
    Write-Host "no target"
    exit
}

# 创建 .stash 文件夹, 用于存储原文件
$stashFolder = Join-Path -Path (Get-Location) -ChildPath ".stash"
if (-not (Test-Path $stashFolder)) {
    New-Item -Path $stashFolder -ItemType Directory | Out-Null
}

$images | ForEach-Object -Parallel {
    $image = $_
    $outputFile = [System.IO.Path]::ChangeExtension($image.FullName, ".avif")
    Write-Host "convert: $($image.Name) -> $outputFile"

    # using ImageMagick
    # -abckground white: remove all alpha channel
    magick $image.FullName -quality 75 -background white -flatten $outputFile

    if (Test-Path $outputFile) {
        Write-Host "success: $outputFile, moved to .stash"
        $stashPath = Join-Path -Path $using:stashFolder -ChildPath $image.Name
        Move-Item -Path $image.FullName -Destination $stashPath -Force
    } else {
        Write-Host "fail: $outputFile"
    }
} -ThrottleLimit 6
