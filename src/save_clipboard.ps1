param (
    [string]$Directory = (Get-Location).Path,
    [string]$Prefix = "img"
)

if ([Threading.Thread]::CurrentThread.ApartmentState -ne 'STA') {
    Start-Process pwsh -ArgumentList @(
        "-STA",
        "-File", "`"$PSCommandPath`"",
        "-Directory", "`"$Directory`"",
        "-Prefix", "`"$Prefix`""
    ) -Wait
    exit
}

Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing

# 获取剪贴板图像
$img = [Windows.Forms.Clipboard]::GetImage()

if ($null -eq $img) {
    Write-Warning "No image in clipboard!!"
    exit 1
}

# 临时 PNG
$temp = [System.IO.Path]::GetTempFileName() + ".png"
$img.Save($temp, [System.Drawing.Imaging.ImageFormat]::Png)

# 输出路径
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$outFile = Join-Path $Directory ("{0}_{1}.avif" -f $Prefix, $timestamp)

# 调用 ImageMagick
magick $temp -quality 85 $outFile

Remove-Item $temp -ErrorAction Ignore

Write-Output $outFile
