## 总目录结构

Linux 遵循 FHS (File Hierarchy Standard) 系统目录标准:

- `/bin`: 系统必须的可执行文件
- `/sbin`: 系统必须的二进制文件, 如系统管理和维护
- `/usr/bin`: 用户可执行文件, 如用户命令
- `/usr/sbin`: 非必要的系统二进制文件, 供系统管理员使用 
- `/usr/local/bin`, `/usr/local/sbin`: 本地安装应用程序, 编译安装时通常在这里. `bin` 和 `sbin` 用于区分用户本地安装和包管理器安装.
- `/lib`, `/usr/lib`: 共享库文件
- `usr/local/lib`: 本地安装的库文件.


linux 目录:

- `/dev/shm`
- `/tmp`
- `/proc`

FHS 规定共享库位置为:
- `/lib` 存放系统基础的共享库, 用于 `/bin, /sbin` 下的库, 还有系统启动.
- `/user/lib` 非系统运行时所需要的关键性库, 主要用于开发时的共享库. 一般不会被用户程序或 shell 脚本直接用到, 还包括开发时可能用到的静态库和目标文件.
- `/user/local/lib` 第三方应用程序的库, 如 python 的共享库在 `/usr/local/lib/python` 下, python.exe 在 `/user/local/bin` 下. 