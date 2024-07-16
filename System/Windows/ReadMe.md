
![](../../attach/Pasted%20image%2020240711165318.png)

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