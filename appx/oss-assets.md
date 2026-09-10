## OSS 图片目录

`E:\til\assets` 通过 rclone + WinFsp 挂载 `ali-oss:yay-waves/til/`。

先在各平台运行 `rclone config`，配置名为 `ali-oss` 的 S3 remote，选择 Alibaba，
设置 OSS endpoint 和有权访问 `yay-waves/til/` 的凭据。脚本复用此配置，不保存 OSS 密钥；
脚本生成的随机密码仅用于本机管理接口。Windows 使用 PowerShell 7.4+，Unix 需要 rclone、jq 和 FUSE。

```powershell
# 启动；已经运行时不会重复挂载。可以在 shell:startup 中配置自启动方式
pwsh -NoProfile -File appx/oss-assets.ps1 -Action Start

# 查看缓存状态和上传队列
pwsh -NoProfile -File appx/oss-assets.ps1 -Action Status

# 在线刷新目录，发现其他设备或 OSS 控制台的修改
pwsh -NoProfile -File appx/oss-assets.ps1 -Action Refresh

# 完整读取所有文件，补齐本地缓存。建议首次配置时用于完整同步 OSS
pwsh -NoProfile -File appx/oss-assets.ps1 -Action Warm

# 先关闭正在使用 assets 的程序，再停止挂载；有待上传文件时拒绝停止
pwsh -NoProfile -File appx/oss-assets.ps1 -Action Stop
```

Unix 对应命令：(Unix 使用 rclone --daemon + FUSE) 

```sh
sh appx/oss-assets.sh start
sh appx/oss-assets.sh status
sh appx/oss-assets.sh refresh
sh appx/oss-assets.sh warm
sh appx/oss-assets.sh stop
```

## 缓存和读写

| 配置 | 值 | 行为 |
| --- | --- | --- |
| VFS 模式 | `full` | 所有读写经过磁盘缓存 |
| 缓存目录 | `E:\til\.oss-assets\cache` | 跨进程重启保留文件内容 |
| 上传延迟 | `5s` | 文件关闭且最后访问后 5 秒开始上传；失败自动重试 |
| S3 指纹 | `--vfs-fast-fingerprint` | 减少打开缓存文件时对远端元数据的查询 |
| 管理端口 | `127.0.0.1:5579` | 仅本机监听，使用随机密码认证 |

本地端口密码由脚本自动生成配置；远程 OSS 鉴权则需要用户自行用 `rclone config` 配置。
VFS 缓存每隔 `1h` 检查一次。日志见 `.oss-assets/mount.log`，默认日志级别为 `INFO`。

短暂断网、睡眠或切换网络时，只要进程仍在、目录缓存有效且文件内容已完整缓存，
就具备离线读取的条件。 目录信息存在内存中，因此重启后纯离线启动仍不保证可用。
较长的目录缓存会推迟发现外部修改；恢复联网后可运行 `Refresh`。

## 参考

* [rclone VFS 缓存](https://rclone.org/commands/rclone_mount/#vfs-file-caching)、
* [rclone 目录缓存](https://rclone.org/commands/rclone_mount/#vfs-directory-cache)、
* [Cloudflare 浏览器与边缘 TTL](https://developers.cloudflare.com/cache/how-to/edge-browser-cache-ttl/)、
