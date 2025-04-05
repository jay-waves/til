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
    # -flatten: 去除透明通道
	magick $image.FullName -resize 75% -quality 50 -background white -flatten $outputFile
	# -chop 0x1 裁掉底部一个像素宽度, 避免绿边. 
	# AVIF 编码器使用 YUV420 二次采样时, 图像高度为奇数, 底部可能出现一像素绿边.
	$imgHeight = (magick identify -format "%h" $outputFile)
	if ($imgHeight % 2 -ne 0) {
		magick $outputFile  -chop 0x1 $outputFile
	}

    if (Test-Path $outputFile) {
        Write-Host "success: $outputFile, moved to .stash"
        $stashPath = Join-Path -Path $using:stashFolder -ChildPath $image.Name
        Move-Item -Path $image.FullName -Destination $stashPath -Force
    } else {
        Write-Host "fail: $outputFile"
    }
} -ThrottleLimit 6
