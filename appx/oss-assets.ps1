#Requires -Version 7.4
param(
    [ValidateSet('Start', 'Status', 'Refresh', 'Warm', 'Stop')]
    [string] $Action = 'Start',
    [switch] $Warm
)
$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
$mount = Join-Path $repo 'assets'
$state = Join-Path $repo '.oss-assets'
$remote = 'ali-oss:yay-waves/til/'
$credentialPath = Join-Path $state 'rc-credential.xml'

function Invoke-Rc([string] $Method, [hashtable] $Body = @{}) {
    $response = Invoke-WebRequest "http://127.0.0.1:5579/$Method" -Method Post -NoProxy -TimeoutSec 30 `
        -Authentication Basic -AllowUnencryptedAuthentication -Credential (Import-Clixml $credentialPath) `
        -ContentType 'application/json' -Body ($Body | ConvertTo-Json -Compress)
    $response.Content | ConvertFrom-Json -AsHashtable
}

function Get-Status {
    $status = Invoke-Rc 'vfs/stats'
    if ($status.fs.TrimEnd('/') -ne $remote.TrimEnd('/')) { throw 'RC belongs to another remote.' }
    $status
}

function Update-Directory {
    $null = Get-Status
    $result = Invoke-Rc 'vfs/refresh' @{ recursive = 'true' }
    if (!$result.ContainsKey('result') -or @($result.result.Values | Where-Object { $_ -ne 'OK' }).Count) {
        throw "Directory refresh failed: $($result | ConvertTo-Json -Compress)"
    }
}

if ($Action -eq 'Start') {
    New-Item -ItemType Directory -Path $state -Force | Out-Null
    $lock = [IO.File]::Open("$state/start.lock", 'OpenOrCreate', 'ReadWrite', 'None')
    try {
        $status = try { Get-Status } catch { $null }
        if (!$status) {
            # WinFsp needs a nonexistent mount point; only remove an ordinary empty directory.
            if (Test-Path -LiteralPath $mount) {
                if ((Get-Item -LiteralPath $mount -Force).Attributes -band [IO.FileAttributes]::ReparsePoint) {
                    throw 'assets is already a mount or link.'
                }
                [IO.Directory]::Delete($mount, $false)
            }
            if (!(Test-Path -LiteralPath $credentialPath)) {
                $password = [Convert]::ToHexString([Security.Cryptography.RandomNumberGenerator]::GetBytes(32))
                [pscredential]::new('til-assets', (ConvertTo-SecureString $password -AsPlainText -Force)) |
                    Export-Clixml -LiteralPath $credentialPath
            }
            $credential = Import-Clixml $credentialPath
            $arguments = @(
                'mount', $remote, $mount,
                '--vfs-cache-mode=full', '--vfs-cache-max-age=8760h',
                '--dir-cache-time=720h', '--poll-interval=0', '--vfs-fast-fingerprint',
                '--contimeout=10s', '--timeout=30s',
                '--rc', '--rc-addr=127.0.0.1:5579',
                '--log-level=NOTICE', '--log-file-max-size=10M', '--log-file-max-backups=2',
                '--cache-dir', "$state/cache", '--log-file', "$state/mount.log"
            )
            $process = Start-Process rclone.exe -WindowStyle Hidden -PassThru `
                -ArgumentList (($arguments | ForEach-Object { '"' + $_ + '"' }) -join ' ') `
                -Environment @{ RCLONE_RC_USER = $credential.UserName; RCLONE_RC_PASS = $credential.GetNetworkCredential().Password } `
                -RedirectStandardError "$state/stderr.log" -RedirectStandardOutput "$state/stdout.log"
            $ready = $false
            for ($i = 0; $i -lt 60; $i++) {
                if ($process.HasExited) { throw "rclone exited; see $state/mount.log and stderr.log" }
                try { $null = Get-Status; $ready = Test-Path -LiteralPath $mount } catch { }
                if ($ready) { break }
                Start-Sleep -Milliseconds 500
            }
            if (!$ready) { throw "Mount readiness timed out; see $state/mount.log" }
            Update-Directory
        }
        Write-Output "Mounted: $mount"
    } finally { $lock.Dispose() }
    if (!$Warm) { return }
    $Action = 'Warm'
}

switch ($Action) {
    'Status' { Get-Status | ConvertTo-Json -Depth 6 }
    'Refresh' { Update-Directory; 'Directory refreshed.' }
    'Warm' {
        Update-Directory
        $files = @(Get-ChildItem -LiteralPath $mount -File -Recurse -Force)
        foreach ($file in $files) {
            $stream = [IO.File]::OpenRead($file.FullName)
            try { $stream.CopyTo([IO.Stream]::Null) } finally { $stream.Dispose() }
        }
        "Cached $($files.Count) files."
    }
    'Stop' {
        $status = Get-Status
        if ($status.diskCache.uploadsQueued -or $status.diskCache.uploadsInProgress -or $status.inUse -gt 1) {
            throw 'Mount is busy; close files and wait for uploads before stopping.'
        }
        $null = Invoke-Rc 'core/quit'
        'Mount shutdown requested; cache retained.'
    }
}
