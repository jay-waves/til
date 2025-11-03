Socket, 套接字提供点对点的双向进程间通信. 套接字仅能和同一域 (Comunication Domain) 中的套接字进行通信, 通信域定义在 `<sys/socket.h>`. Unix Domain 提供了套接字地址空间, 不同系统间通信被称为 Internet Domain, 在 Unix Domain 中的套接字被称为 Unix Paths. 

Internet Domain 使用 TCP/IP 网络协议进行通信. 套接字有如下几种类型:

| Socket Type       | 通信    | 面向连接 | 可靠传输                           | 协议           | 消息边界 | 类型名           | 长度限制                    |
| ----------------- | ------- | -------- | ---------------------------------- | -------------- | -------- | ---------------- | --------------------------- |
| Stream            | Two-way | Yes      | Sequenced, No loss                 | TCP            | No       | `SOCK_STREAM`    | No, stream-based            |
| Datagram          | Two-way | No       | No guarantee of order and delivery | UDP            | Yes      | `SOCK_DGRAM`     | Yes, limited by packet size |
| Sequential Packet | Two-way | Yes      | Sequenced, No loss                 | SCTP (uncomon) | Yes      | `SOCK_SEQPACKET` | Yes, packet-based           |
| Raw               |         |          |                                    | IP             |          |                  | Yes, limited by packet size |

## SOCK_SRTEAM

- server: `socket()` -> `bind(addr)` -> `listen()` -> `accept()` -> waiting for client
- client: `socket()` -> `connect(addr)` 

创建绑定套接字:
```c
// 创建一个 socket, 返回描述符
int socket(int domain, int type, int protocol);

// 将一个路径 (in Unix Domain) 或者网络地址 (in Internet Domain) 
// 绑定到一个 socket. 
int bind(int s, const struct sockaddr *name, int namelen);
```

```c
// for Unix Domain, path <= 14 characters
#include <sys/socket.h>

// POSIX.1g 
typedef unsigned short int sa_family_t;
struct sockaddr {
	/*
		AF_LOCAL: 本地地址 
		AF_INET:  IPv4
		AF_INET6: IPv6
		
		PF_XXX 和 AF_XXX 的值保持一致, AF_XXX 用于初始化 地址族, 
		PF_XXX 用于初始化 套接字.
	*/
	sa_family_t sa_family; // 地址族
	char sa_data[14];      // 地址的具体值
};

bind (sd, (struct sockaddr *) &addr, length);


// for Unix Domain requires more characters
#include <sys/un.h>

struct sockaddr_un {
	sa_family_t sun_family; // AF_UNIX 
	char sun_path[108];     // 
};
bind (sd, (struct sockaddr_un *) &addr, length);


// fro Internet Domain
#include <netinet/in.h>

struct sockaddr_in 
{
	sa_family_t sin_family;  // 16b, AF_INET
	in_port_t sin_port;      // 16b 
	struct in_addr sin_addr; // IPv4 address	
	...
};

bind (sd, (struct sockaddr_in *) &addr, length);
```

### 握手过程

```c
// 服务器初始化:
int listen(int s, int backlog);
// 阻塞等待客户端, 从而拿到客户端地址
int accept(int listensockfd, struct sockaddr *cliaddr, socklen_t *addrlen);

// 初始化一个客户端, 和服务器三次握手
int connect(int s, struct sockaddr *servaddr, int addrlen);

// for Unix Domain call
strcut sockaddr_un server;
connect (sd, (struct sockaddr_un *)&server, length);

// for Internet Domain call
struct sockaddr_in;
connect (sd, (struct sockadr_in *)&server, length);
```

`connect()` 可能有几种返回码:
- 三次握手无法建立. 即发出的 SYN 包没有收到响应, 返回 TIMEOUT
- 收到 RST 响应. TCP RST 产生的条件是: SYN 抵达, 但该端口上没有监听; 或 ...
- 不可达响应 (Connection Refused). 即目的地不可达, 通常是路由不通.

### 数据传输过程

数据传输:
```c
write()

/*
	读取数据, 最多读取 sz 大小. 没有数据时, 可能提前返回.
	
	如果返回值大于 0, 代表成功读取的字节数.
	如果返回值是 0, 代表 read EOF. 在 TCP 下, 意味着对端发送了 FIN. 
	如果返回值是 -1, 表示出错. 
*/
ssize_t read(int socketfd, void* buffer, size_t sz);

/*
	send() 是异步的, 直到所有字节流发送完再返回. 除非有错误.
	但是, "发送" 并不是发送到对端, 而是指拷贝到了系统的发送缓冲区, 接下来由内核后台接管.
	
	如果内核发送失败, 需要其他机制来监控. 但更推荐对端应用层主动发送确认消息.
*/
// send and recv have operational `flags`, formed from biwaise OR of 
// zero or more the following:
// `MSG_OOB`: out-of-band
// `MSG_DONTROUTE`: turn on `SO_DONTROUTE`, only for diagnostic
// `MSG_PEEK`: peek data present on socket, but not consumed, so a 
//  subsequent receive oepration will see the same data.
int send(int s, const char *msg, int len, int flags)
int recv(int s, char *buf, int len, int flags)
```

TCP 一旦断开连接 (一端下线), 下次必须重新握手.

### 挥手过程

用 `close()` 直接进行四次挥手, 关闭双向链接.
```c
/*
	将套接字引用计数减一. 当没有引用时, 释放套接字, 并关闭数据流.
*/
int close(int sockfd);
```

另一种关闭方式是: 客户端关闭写方向, 然后允许服务器发送最后的收尾消息, 不立刻关闭套接字的读方向. 如果客户端直接调用 `close()`, 内核会立刻将套接字标记为"不可读", 后续接收到任何服务端数据, 都会回送一个 `RST` 而不是 `ACK`.

```c
/*
	@howto:
		SHUT_RD(0): 关闭读方向. 丢弃缓冲区已有数据. 此后任何读, 都会返回 EOF. 
		当接收到对端数据时, 还是会发送 ACK...
		SHUT_WR(1): 关闭写方向. 立刻将缓冲区发送给对端, 然后追加一个 FIN. 此后, 
		不允许用户再写入.
		SHUT_RDWR(2): RD + WR
	
	shutdown 没有引用计数这个概念, 会立刻发出 FIN, 并使套接字不可用. 
	shutdown 不会立刻释放掉套接字和所有资源.
*/
int shutdown(int sockfd, int howto);
```

### 错误处理

当 Socket 已经被关闭, 向其 `write()` 后是不合法的操作, 将直接触发 `SIGPIPE` 信号. 是一种比返回码更强的处理机制. 而 `read()` 通过返回码来返回异常, 因为其

- `write()` 异常时, 直接触发 `SIGPIPE` 信号. 因为用户主动执行了一个非法操作
- `read()` 异常时, 返回负值错误码. 一般由于外部问题导致的异常, 需要用户处理.

#### 网络中断

网络中断时, 如果有路由器回送 ICMP 不可达报文, `read(), write()` 都会退出, 返回 `Unreachable`. 在没有 ICMP 报文时, TCP Socket 无法主动感知到异常. 此时, 只能通过设置 `read()` 超时, 来避免持续阻塞.

例外情况是, 如果 TCP Socket 仍有数据要发送, 但由于网络中断, 多次重传后仍失败, 就会标记该连接异常. 此时, `read()` 调用会自动返回 `TIMEOUT`, 后续 `write()` 会返回 `SIGPIPE` 信号.

```c
// 设置 TCP 超时
// 此后, 如连接超时, recv, read 会从阻塞中返回错误码 EAGAIN / EWOULDBLOCK

struct timeval tv;
tv.tv_sec = 5;
tv.tv_usec = 0;

setsockopt(connfd, SOL_SOCKET, SO_REVTIMEO, (const char*)&tv, sizeof(tv));

// 另外一种方案是, 利用多路复用技术.
```

#### 对端系统崩溃

如果对端系统崩溃, 来不及发出 `FIN` 正常关闭连接, 也需要超时机制来处理. 

如果对端崩溃后又重启, 收到了旧连接的报文, 会返回 `RST` 重置报文. 也会让本机的 `read()` 返回 `Connection Reset` 错误, `write()` 返回 `SIGPIPE` 信号.

#### 数据有效性

## AF_LOCAL 

注意, `127.0.0.1` 属于 `AF_INET` 地址族, 和这里的 `AF_LOCAL` 不是一回事. 本地回环套接字 (Loopback Socket) 是网络套接字, 会通过完整 TCP/IP 协议栈发包解包, 因此比 `AF_LOCAL` (Unix 域套接字) 更慢. `AF_LOCAL` 在 windows 上不支持, 可以考虑 Windows 双工管道作为替代.

```c
int listenfd， connfd；
socklen_t clilen；
struct sockaddr_un cliaddr, servaddr;
listenfd = socket(AF_LOCAL, SOCK_STREAM, 0);
if (listenfd < 0) {
	error(...);
}

servaddr.sun_family = AF_LOCAL;
strcpy(srrvaddr.sun_path, "/tmp/mysocket");
bind(listenfd, (struct sockaddr*)&servaddr, sizerof(servaddr));
```

错误类型:
- 没有服务端绑定到一个本地套接字 (表现为一个文件)
- 服务器在一个无权限的文件下监听
- 

## SOCK_DGRAM

数据报套接字不需要建立连接. 

- server: `socket()` -> `bind(addr)` -> `recvfrom()`
- client: `socket()` -> `sendto()`

```c
int fd;
fd = socket(AF_INET, SOCK_DGRAM, 0);

struct sockaddr_in addr;
addr.sin_family = AF_INET;
addr.sin_addr.s_addr = htol(INADDR_ANY);
addr.sin_port = htons(SERV_PORT);

// could use connect() and bind(), but accept() and listen() are not used
bind(fd, &addr, sizeof(addr));
// UDP 未连接到服务器时, 会持续阻塞; 而 TCP 会返回错误.
/* 
	推荐将 UDP 也绑定到地址 (包含一些上下文), 便于操作系统追踪该链接的上下文.
	否则难以调试, 性能也不高 (每次收发都连接/断开套接字).
*/
connect(...);
```

### 发送数据

```c
#include <sys/socket.h>

sendto()
sendmsg()
sendto()

/*
	UDP 是没有上下文的. 
*/
ssize_t recvfrom(int sockfd, void*buff, size_t nbytes, int flags, 
		struct sockaddr* from, socklen_t *addrlen);

ssize_t sendto(int sockfd, const void* buff, size_t nbytes, int flags, 
		const struct sockaddr* to, socklen_t *addrlen);

recvmsg()
recv()
```

使用 `sendto(), recvfrom()` 时, 推荐将地址相关值置零. 因为该函数有兼容性问题, Linux 上的 to 和 from 地址信息被默认忽略. 



## Socket Options

获取配置: getsockopt() 
