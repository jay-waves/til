## 渲染

早期的图形接口:
- Windows MFC/ATL --> `gdi32.dll`, 用于一般软件窗口. Windows 自己负责的, 在内核里.
- Linux GTK/QT --> `X`, 用于一般软件窗口. 由开源组织 XFree86 独立负责
- Windows DirectX/OpenGAL, 专用于游戏, 由 GPU 渲染.

现代图形接口:
- Windows Vista --> GDI+ --> Direct2D --> DirectX. 统一了软件和游戏的渲染.
- Linux AIGLX --> 用 `OpenGL` 调用 GPU 渲染, 但沿用 X 的架构. 
- Linux Wayland (更新) 

更新的图形接口: 都是先找 GPU 渲染, 然后交给窗口管理器显示. 
- Linux Wayland, 不再用 X.  
- Windows DirectX 

除了 OpenGL 外, 图形渲染 API 还有 Vulkan 和 Apple Mentle.

## 事件响应库

处理窗口或者输入事件. 一般使用 GLFW. 

- Windows 上, 使用 Win32 API 
- Linux 上, 使用 X11, Wayland 窗口系统