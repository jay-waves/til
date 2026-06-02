## Windows API

Windows 将 导出库 (DLL), 导出函数的声明头文件, 相关文件及工具打包为 **SDK (Software Development Kit)**. 通过 Visual Studio 后, MSVC 头文件中的 `Windows.h` 包含了核心 Windows API.

> Windows API 版本使用最广泛的是 *Win32*, Win64 与其相比也没有大变化. 

| 类别         | DLL            | 常见 API                                       | 说明                                                    |
| ------------ | -------------- | ---------------------------------------------- | ------------------------------------------------------- |
| 基本服务     | `kernel.dll`   | `CreateProcess(), ReadFile(), HeapAlloc()`     | 文件系统, 设备, 进程线程, 内存, 错误处理 (基础操作系统) |
| 图形接口设备 | `gdi32.dll`    | `CreateDC(), TextOut(), BitBlt()`              | 图形, 绘图, 打印机                                      |
| 用户接口     | `user32.dll`   | `CreateWindow(), GetMessage(), SendMessage()`  | 与 Windows 窗口交互, 鼠标, 基本控件                     |
| 高级服务     | `advapi32.dll` | `RegOpenKeyEx(), CreateService(), LogonUser()` | 内核额外功能, 如注册表, 服务, 用户账户管理              |
| 通用对话框   | `comdlg32.dll` | `GetOpenFileName(), Printlg(), ChooseFont()`   | 系统通用对话框, 如选择文件框, 打印窗口, 选择字体/颜色框 |
| 通用控件     | `comctl32.dll` | `CreateStatusWindow(), CreatToolbar()`         | 高级控件, 如状态栏, 进度条或工具条.                     |
| Shell        | `shell32.dll`  | `ExtracIcon(), ShellExecute()`                 | 与图形 Shell 相关的操作                                 |
| 网络服务     | `ws2_32.dll`   | `send(), recv()`                               | 网络如 Winsock, NetDDE, RPC, NetBIOS                                                        |

部分类别的 Windows API 功能很原始, Windows 还提供了很多应用模块. 如 Internet 模块, OpenGL 模块, ODBC 模块, WIA 模块等.


## Windows 开发框架

| 框架                | 简介                               | 用途                      |
| ------------------- | ---------------------------------- | ------------------------- |
| MFC                 | Miscorsoft Foundation Classes      | 开发 Windows GUI 应用程序, 是底层 WinAPI 的封装 |
| ATL                 | Active Template Library            | 开发 COM (组件对象模型) 组件和轻量级 ActiveX 控件. 比 MFC 更轻量 |
| MSVC                | Microsoft Visual C++ 编译工具链    | 编译器 cl.exe, 调试器 msvcdbg.exe, 链接器 link.exe    |
| MSVCRT              | Microsfot Visual C++ Runtime       | 标准库和运行时            |
| WinAPI (SDK)        | Windows APT, 原始 Windows 系统调用 | 最底层的系统调用接口, 包括基础 GUI 元素 (和 Unix 系统调用不同)      |
| .NET (WPF/WinForms) | 更高级的 C# 与 BV.NET 应用开发框架 | 现代 GUI 和服务                          |
