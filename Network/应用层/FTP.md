FTP (File Transfer Protocol), TFTP, SFTP v1/v2 用来远程传输文件. FTP 协议定义于 [RFC 959](https://datatracker.ietf.org/doc/html/rfc959), 是应用层协议, 是客户端/服务器架构的协议, 但是支持双向数据传输. FTP 的目标是促进文件共享, 屏蔽不同文件系统的细节, 建立可靠和高效的文件传输方式.

## 连接模式

### 主动连接模式

数据传输前, 建立两个 [TCP](../传输层/TCP.md) 连接, FTP 服务器的端口分别为 20 和 21. 21 端口是命令传输端口, 20 端口为数据传输端口.

客户端开启 N 和 N+1 两个端口 (端口选取通常接近), N 为客户端的命令端口, N+1 为客户端的数据端口.

1. 客户端用 N 端口连接 FTP 服务器的 21 端口, 建立**控制连接**并通知.
2. FTP 服务器使用数据端口 20 **主动**连接客户端的 N+1 端口.

主动模式下, FTP 服务器的[ReadMe](Network/防火墙/ReadMe.md)只需要开启 21 端口的准入, 而客户端需要开启 20 端口的准入.

### 被动连接模式

1. 客户端开启 N 和 N+1 端口, 用 N 端口连接 FTP 服务器的 21 端口, 发送 `PASV` 命令, 让服务器使用被动模式
2. *控制连接*建立后, 服务器开启一个数据端口 P, 然后用 `PORT` 命令将端口号 P 告知客户端
3. 客户端用 N+1 端口连接服务器的数据端口 P, 建立*数据连接*.

客户端的[ReadMe](Network/防火墙/ReadMe.md)通常"准出", 因此不用多余配置.

## 控制命令

```
ABOR                    放弃之前的命令和数据传输
LIST filelist
PORT n1,n2,n3,n4,n5,n6  客户端定位: IP (n1.n2.n3.n4) : PORT (n5*256+n6)
QUIT                    退出
RETR filename           取文件
STOR filename           放文件
SYST                    系统类型 (Win, Unix)
TYPE type               文件类型, A 表示纯文本, I 表示二进制
USER username
...
```

## 数据传输模式

- ASCII 模式: 用于传输纯文本
- BINARY 模式: 图片或大文件

在 ASCII 模式下, FTP 会对文件内容进行一些转换, 主要是针对不同平台下换行符的处理 (Windows `CRLF`, Unix `LF`). BiNARY 模式则会尊重原始内容.

## 使用方法

使用 `aaa` 配置 FTP 服务器: 用户名密码, 用户可访问目录, 超时时间等.


使用 `ftp` 来建立控制连接, 进入客户端视图. 默认传输方式为 ASCII, 使用 `binary` 修改传输模式为 BINARY. 使用 `get` 来获取文件到本地.

`ftp` 是 Windows 自带的命令, 详细子命令见[微软文档](https://learn.microsoft.com/en-us/previous-versions/orphan-topics/ws.10/cc755356(v=ws.10)). 常用命令行命令:
- `ftp 122.212.444.2`, or `ftp ftp.gnu.org`
- `ftp>dir`, `ftp>cd`, `ftp>ls`, ...
- `ftp>binary` 文件传输模式为 BiNARY
- `ftp>ascii` 文件传输模式为 ASCII
- `ftp>delete`
- `ftp>get`, `ftp>mget` 复制服务器文件到本地
- `ftp>put` 复制本地文件到服务器
