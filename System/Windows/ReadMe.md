
![](../../attach/Pasted%20image%2020240711165318.avif)

`C:` 系统分区
- `C:\Windows` 操作系统的核心文件和库文件.
	- `C:\Windows\System32` 存放系统DLL文件, 驱动和系统服务. 如 Windows 内核 `ntoskrnl.exe`
- `C:\Program Files`, `C:\Program Files (x86)` 存放 64 (32) 位程序.
- `C:\Users` 用户文件夹
	- `C:\Adminis` 管理员文件夹
	- `C:\y'j'w` 用户文件夹
		- `C:\y'j'w\AppData\Local` 用户**本地计算机上**的应用程序数据/配置
		- `C:\y'j'w\Appata\Local\Temp` 临时文件, 缓存文件
		- `C:\y'j'w\AppData\LocalLow` 存放应用程序的低权限数据
		- `C:\y'j'w\AppData\Roaming` 需要**多台机器同步**的用户配置/数据.
- `C:\ProgramData` 用户**公用**的数据或设置

## Windows 开发框架

| 框架                | 简介                               | 用途                      |
| ------------------- | ---------------------------------- | ------------------------- |
| MFC                 | Miscorsoft Foundation Classes      | 开发 Windows GUI 应用程序, 是底层 WinAPI 的封装 |
| ATL                 | Active Template Library            | 开发 COM (组件对象模型) 组件和轻量级 ActiveX 控件. 比 MFC 更轻量 |
| MSVC                | Microsoft Visual C++ 编译工具链    | 编译器 cl.exe, 调试器 msvcdbg.exe, 链接器 link.exe    |
| MSVCRT              | Microsfot Visual C++ Runtime       | 标准库和运行时            |
| WinAPI (SDK)        | Windows APT, 原始 Windows 系统调用 | 最底层的系统调用接口, 包括基础 GUI 元素 (和 Unix 系统调用不同)      |
| .NET (WPF/WinForms) | 更高级的 C# 与 BV.NET 应用开发框架 | 现代 GUI 和服务                          |
