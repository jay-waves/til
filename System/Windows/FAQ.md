## 查看设备序列号

使用 [WMIC](WMIC.md) 命令行: `wmic bios get serialnumber`

## 查看 Windows 系统信息

搜索[^1] `WinVer`, 或搜索 `系统信息`

[^1]: 这里的搜索都是指 Win+R 唤起 "Windows 运行".

## 启动 Windows 安全模式

登录时, 摁住 Shift, 重新启动.

进入安全模式后, 搜索 MsConfig, 进行系统级设置.

## 显示器设置

搜索 `dxdiag`, 使用 DirectX 诊断工具

## 编辑环境变量

搜索: sysdm.cpl

## 管理服务

"Windows Service" 概念, 类似 "Linux Daemon". 都是后台的守护进程.

- 启动服务 `net start [name]`
- 停止服务 `net stop [name]`
- 查询某个服务 `sc query [name]`
- 查看运行中的程序 `net start`
- 删除服务 `sc delete [name]`
