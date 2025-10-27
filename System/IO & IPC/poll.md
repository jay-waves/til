## poll

`poll()` 是另一种 [I/O 复用方式](select.md). `poll()` 解决了 `select()` 的文件描述符有限的缺点.

```c
struct pollfd {
	int fd;        // file descriptor
	short events;  // events to look for
	short revents; // events returned
};

/*
	timeout:
	- timeout < 0, 阻塞
	- timeout = 0, 不阻塞, 立即返回
	- timeout > 0, 等待指定毫秒数后返回
*/
int poll(struct pollfd *fds, unsigned long nfds, int timeout);

// 定义一些请求事件掩码:
#define POLLIN 0x0001 // any readable data available
#define POLLRI 0x0002 // OOB/URG readable data
#define POLLOUT 0x0004 // fd is writeable
#define POLLRDNORM 0x0040 // non-OOB/URG data available 
#define POLLRDBAND 0x0080 // OOB/URG readable data
#define POLLWRNORM POLLOUT 
#define POLLWRBANED 0x0111 // OOB/URG data can be written

// 只在 revents 中存在的事件
#define POLLERR 0x0008
#define POLLHUP 0x0010
#define POLLNVAL 0x0020
```

`poll` 每次检测事件之后, 不会修改 `events` 掩码, 而是将结果写到 `revents` 字段. 

### 实例

```c
struct pollfd event_set[INIT_SIZE];
event_set[0].fd = listen_fd;
event_set[0].events = POLLRDNORM;

int i; 
for (i = 1; i < INIT_SIZE; i++) {
	event_set[i].fd = -1; // 表示未占用
}

for(;;) {
	if ((ready_num = poll(event_set, INIT_SIZE, -1)) < 0 ) {
		error (...)
	}
	
	if (event_set[0].revents & POLLRDNORM) {
		socklen_t client_len = sizeof(client_addr);
		conn_fd = accept(listen_fd, (struct sockaddr*)&client_addr, &client_len);
		// 将该套接字记录在 events 监听
		for (i = 1; i < INT_SIZE; i++) {
			if (event_set[i].fd < 0) {
				event_set[i].fd = conn_fd;
				event_set[i].events = POLLRDNORM;
				break;
			}
		}
	}
			
	if (i == INIT_SIZE) {
		error(...)
	}
	
	if (--ready_num <= 0)
		continue;
		
	for (i = 1; i < INIT_SIZE; i++) {
		int socket_fd; 
		if ((socket_fd = event_set[i].fd) < 0) 
			continue;
			
		if (event_set[i].revents & (POLLRDNORM | POLLERR)) {
			if ((n = read(socket_fd, buf, MAXLINE)) > 0) {
				...
			} else if (n = 0 || errno == ECONNRESET) {
				close(socket_fd);
				event_set[i].fd = -1;
			} else {
				error(...);
			}
			
			if (--ready_num <= 0)
				break;
		}
	}
}
```

## epoll 

