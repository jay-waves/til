## 总目录结构

Linux 遵循 FHS (File Hierarchy Standard) 系统目录标准:

- `/bin`: 基础可执行命令. 如 cd, ls, mkdir 
- `/sbin`: 系统可执行命令, 多涉及系统管理. 如 modprobe, hwclock, ifconfig.
- `/dev` 设备文件存储目录
- `/etc` 系统配置文件目录. 
- `/mnt` 挂载存储设备目录.
- `/opt` 可选, 一些厂商的大型软件包会安装在这.
- `proc` 存储系统运行时信息, 是内存中文件系统.
- `/tmp` 临时文件
- `/var` 变动目录, 存储日志等.
	- `/var/log` 日志
- `/sys` linux 2.6 之后, 内核 sysfs 系统挂载到该目录, 包含总线 / 驱动 / 设备等.
- `/usr`
	- `/usr/bin`: 用户可执行文件, 如用户命令和编程环境. 比如 gcc 在这里有一个软链接. 
	- `/usr/sbin`: 非必要的系统二进制文件, 供系统管理员使用 
	- `/usr/share`: 共享的一些资源数据
	- `/usr/include`: 用户开发环境的头文件
	- `/usr/local`: 用户本地安装的应用程序
		- `/usr/local/bin` 用户自行编译的程序，通常 `make install` 到这里
		- `/usr/local/sbin` 一些包管理器会把程序安装在这里
		- `/usr/local/lib` 用户自行安装的库文件
- `/lib`, `/usr/lib`: 共享库文件
- `/home` 个人目录
- `/root` 管理员目录


FHS 规定共享库位置为:
- `/lib` 存放系统基础的共享库, 用于 `/bin, /sbin` 下的程序, 还有系统启动.
- `/user/lib` 非系统运行时所需要的关键性库, 主要用于开发时的共享库 / 静态库 / 目标文件.
- `/user/local/lib` 第三方应用程序的库, 如 python 的共享库在 `/usr/local/lib/python` 下, python.exe 在 `/user/local/bin` 下. 



