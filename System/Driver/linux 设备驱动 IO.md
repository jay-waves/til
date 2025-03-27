## 阻塞 IO

*阻塞操作*是指: 设备操作无法立即获得资源, 其进程被挂起, 进入睡眠状态. 
*非阻塞操作* 则并不挂起进程, 要么放弃 (返回 `-EAGAIN`), 要么不停地查询.

阻塞读取:
```c
char buf;
fd = open("/dev/ttyS1", O_RDWR);

res = read(fd, &buf, 1);
if (res == 1)
	...
```

非阻塞读取:
```c
char buf;
fd = open("/dev/ttyS1", O_RDWR | O_NONBLOCK);

while(read(fd, &buf, 1) != 1)
	continue;
...
```

除了 `open()` 调用, `ioctl()`, `fcntl()` 也可以更改读写方式, 如从阻塞变为非阻塞.

### 等待队列

使用*等待队列 (Wait Queue)* 来使阻塞进程等待资源, 适时唤醒.

```
      +----------------------------------------------------------------+
      |                                                                |
      v                                                                |
wait_queue_head_t ----> wait_queue ----> wait_queue ----> wait_queue --+
   (lock)              (task_struct)     (task_stuct)     (task_struct)
                       (func)            (funct)          (func)
```

```c
// in include/linux/wait.h

typedef struct __wait_queue_head {
	spinlock_t       lock;
	struct list_head head; // in include/linux/list.h
} wait_queue_head_t;

typedef struct wait_queue_entry {
	unsigned int flags;
	void *private;
	wait_queue_func_t func;
	struct list_head entry;
} wait_queue_entry_t;


wait_queue_head_t my_queue; // 元素
init_waitqueue_head(&my_queue); // DECLARE_WAIT_QUEUE_HEAD (name)


void add_wait_queue(wait_queue_head_t *q, wait_queue_entry_t *wait);
void remove_wait_queue(wait_queue_head_t *q, wait_queue_entry_t *wait);
```

等待事件并唤醒队列:

```c
wait_event(queue, condition)
wait_event_interruptible(queue, condition) // 可以被信号打断
wait_event_timeout(queue, condition, timeout) // timeout 以 jiffy 为单位
wait_event_interruptible_timeout(queue, condition, timeout)

void wake_up(wait_queue_head_t *queue); // 应该与 wait_event 成对使用
void wakt_up_interruptible(wait_queue_head_t *queue);
```

将进程状态设置为 `TASK_UNINTERRUPTIBLE`, 并挂到等待队列, 直到资源可获得:

```c
sleep_on(wait_queue_head_t *q);
interruptible_sleep_on(wait_queue_head_t *q);

// examples 
DECLARE_WAITQUEU(wait, current);
add_wait_queue(&xxx_wait, &wait);

do {
	avail = device_writable(...);
	if (avail < 0) {
		if (file->f_flags & O_NONBLOCK) { // 非阻塞
			ret = -EAGAIN;
			goto out;
		}
		__set_current_state(TASK_INTERRUPTIBLE); // 改变进程状态
		schedule();                              // 调度其他进程执行
		if (signal_pending(current)) {           // 如果是被信号唤醒
			ret = -ERESTARTSYS;
			goto out;
		}
	}
} while (avail < 0);

device_write(...); // 写设备缓冲区

out: 
	remove_wait_queue(&xxx_wait, &wait);
	set_current_state(TASK_RUNNING);
	return ret;
```

## 轮询

## 异步 IO

### Linux AIO

### glibc AIO

