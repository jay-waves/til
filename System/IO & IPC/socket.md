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
// 阻塞等待客户端. 
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
- 不可达响应. 即目的地不可达, 通常是路由不通.

### 数据传输过程

数据传输:
```c
write()

/*
	读取数据, 最多读取 sz 大小. 没有数据时, 可能提前返回.
	
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


## SOCK_DGRAM

数据报套接字不需要建立连接. 

- server: `socket()` -> `bind(addr)` -> `recvfrom()`
- client: `socket()` -> `sendto()`

```c
// could use connect() and bind(), but accept() and listen() are not used
bind()
connect()
```

### 发送数据

```c
#include <sys/socket.h>

sendto()
sendmsg()
sendto()

ssize_t recvfrom(int sockfd, void*buff, size_t )
recvmsg()
recv()
```

## Socket Options

获取配置: getsockopt() 
