$ErrorActionPreference = 'Stop'
$files = @(git ls-files '*.md' '*.typ' '*.html' '*.css')
$rx = [regex]'(?i)(?:\.\./|/)*assets/(?<p>[^"''<>\s\)\]\}?#]+)'
$refs = @{}
foreach ($f in $files) {
    $path = Join-Path (Get-Location) $f
    if (!(Test-Path -LiteralPath $path)) { continue }
    $text = [IO.File]::ReadAllText($path)
    foreach ($m in $rx.Matches($text)) {
        $name = Split-Path ([uri]::UnescapeDataString($m.Groups['p'].Value).Replace('/', '\')) -Leaf
        if (!$refs.ContainsKey($name)) { $refs[$name] = [Collections.Generic.HashSet[string]]::new() }
        $parts = $f -split '[\\/]'
        $key = if ($parts.Count -gt 2) { "$($parts[0])\$($parts[1])" } elseif ($parts.Count -eq 2) { "$($parts[0])\root" } else { 'shared\root' }
        [void]$refs[$name].Add($key)
    }
}
$map = @{}
foreach ($item in Get-ChildItem assets -File) {
    [array]$targets = if ($refs.ContainsKey($item.Name)) { @($refs[$item.Name]) } else { @('_unreferenced\legacy') }
    $target = if ($targets.Count -eq 1) { $targets[0] } else { 'shared\shared' }
    $map[$item.Name] = "$target\$($item.Name)"
}
$utf8 = [Text.UTF8Encoding]::new($false)
foreach ($f in $files) {
    $path = Join-Path (Get-Location) $f
    if (!(Test-Path -LiteralPath $path)) { continue }
    $text = [IO.File]::ReadAllText($path)
    $new = $text
    foreach ($name in $map.Keys) {
        if ($refs.ContainsKey($name)) { $new = $new.Replace('assets/' + $name, 'assets/' + $map[$name].Replace('\', '/')) }
    }
    if ($new -cne $text) {
        try { [IO.File]::WriteAllText($path, $new, $utf8) }
        catch { Write-Warning "Could not update $f`: $($_.Exception.Message)" }
    }
}
foreach ($item in @(Get-ChildItem assets -File)) {
    $dest = Join-Path assets $map[$item.Name]
    $null = New-Item -ItemType Directory -Path (Split-Path $dest -Parent) -Force
    Move-Item -LiteralPath $item.FullName -Destination $dest
}
[pscustomobject]@{ SourceFiles = $files.Count; ReferencedNames = $refs.Count; Moved = $map.Count; RemainingFlat = (Get-ChildItem assets -File).Count } | ConvertTo-Json
