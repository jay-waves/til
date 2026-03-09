## 总目录结构

Linux 遵循 FHS (File Hierarchy Standard) 系统目录标准:

- `/bin`: 基础可执行命令. 如 cd, ls, mkdir
- `/sbin`: 系统可执行命令, 多涉及系统管理. 如 modprobe, hwclock, ifconfig.
- `/dev` 设备文件存储目录
- `/etc` 系统配置文件目录. 
- `/mnt` 挂载存储设备目录.
- `/opt` 可选, 一些软件包会安装在这.
- `proc` 存储系统运行时信息, 是内存中文件系统.
- `/tmp` 临时文件
- `/var` 变动目录, 存储日志等.
- `/sys` linux 2.6 之后, 内核 sysfs 系统挂载到该目录, 包含总线 / 驱动 / 设备等.
- `/usr/bin`: 用户可执行文件, 如用户命令
- `/usr/sbin`: 非必要的系统二进制文件, 供系统管理员使用 
- `/usr/local/bin`, `/usr/local/sbin`: 本地安装应用程序, 编译安装时通常在这里. `bin` 和 `sbin` 用于区分用户本地安装和包管理器安装.
- `/lib`, `/usr/lib`: 共享库文件
- `usr/local/lib`: 本地安装的库文件.


FHS 规定共享库位置为:
- `/lib` 存放系统基础的共享库, 用于 `/bin, /sbin` 下的程序, 还有系统启动.
- `/user/lib` 非系统运行时所需要的关键性库, 主要用于开发时的共享库 / 静态库 / 目标文件.
- `/user/local/lib` 第三方应用程序的库, 如 python 的共享库在 `/usr/local/lib/python` 下, python.exe 在 `/user/local/bin` 下. 
