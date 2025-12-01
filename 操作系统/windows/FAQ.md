## 查看设备序列号

使用 [WMIC](WMIC.md) 命令行: `wmic bios get serialnumber`

## 查看 Windows 系统信息

搜索[^1] `WinVer`, 或搜索 `系统信息`

[^1]: 这里的搜索都是指 Win+R 唤起 "Windows 运行".

## 启动 Windows 安全模式

登录时, 摁住 Shift, 重新启动.

进入安全模式后, 搜索 MsConfig, 进行系统级设置.

## 刷新显卡驱动

`win + ctrl + shift + B`

## 驱动设置

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

## 审计

在 本地组策略 -> Windows 设置 -> 本地策略 -> 审核策略 中, 开启 "审核对象访问", 来开启对文件的审计功能.

![|700](https://jay-waves.oss-cn-beijing.aliyuncs.com/til/本地策略组编辑器.avif)

开启 文件属性 -> 安全 -> 高级 -> 审核, 新建审核, 为该文件选择对应的审计属性.

![|700](https://jay-waves.oss-cn-beijing.aliyuncs.com/til/开启文件夹审核.avif)

在事件查看器 `eventvwr` 查看 Windows 日志 -> 安全, 相关事件 ID 包括:
- 4663: 对文件或文件夹尝试访问
- 4656: 对文件的操作请求
- 4670: 更改文件的权限