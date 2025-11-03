| `queue`     | `subprocess` |
| ----------- | ------------ |
| `threading` | `asyncio`             |

CPython 为确保抢占式线程不会破坏解释器对资源的管理, 使用 GIL (Global Interpreter Lock) 确保线程安全, 同一时间只能一个线程执行 Python 字节码, 所以 Python 是 **并发 (concurrency)**, 而不是并行 (parallelism).

对于 IO 密集型任务 (文件IO或网络请求), 线程可以释放 CPU 使用权 (GIL) 来等待任务完成, 所以效率较高. 但对 CPU 密集任务, 多线程不会加速反而带来额外上下文切换开销. 

解决方法有: 使用 Jython/IronPython 这样的无 GIL 的 Python 实现, 或使用非 GIL 依赖库 (如 `numpy`, `concurrent.futures`).

|  Python Mode:                  | Multi-Threads        | Multi-Processes | Coroutines |
| ------------------ | -------------------- | --------------- | ---------- |
| Python Interpreter | share                | multi           |            |
| mode               | Concurrency          | Parallelism     |   **Concurrency**         |
| Applicable Scene   | I/O, Network | Computing       | I/O, Network           |
| CPU Core           | one                  | multi           |           |
| Python Speedup     | no                   | yes             |            |
| Memory Consumed    | 8MB (per stack)                |   10~30MB (per interpreter)            |  <1KB (per call)        |

更多并发知识可参考: [Go/并发](../go/Go%20并发.md), [OS/进程与线程](../../操作系统/process/进程与线程.md)

## 多进程

```python
def run_openssl(data):
	env = os.environ.copy()
	env['password'] = b'123345'
	proc = subprocess.Popen(
		['openssl', 'enc', '-des3', '-pass', 'env:password'],
		env=env,
		stdin=subprocess.PIPE,
		stdout=subprocess.PIPE
		)
	proc.stdin.write(data)
	proc.stdin.flush() # remove stdin cache, ensure openssl gets input.
	return proc
```

## 多线程
```python
def myfunc(arg1, arg2, ...):
	pass

for _ in range(5):
	args = (..., ...)
	thread = Thread(target=myfunc, args=args)
	thread.start()

for thread in threads:
	thread.join()
```

### Lock

尽管由于 Python [GIL](python%20并发.md) 限制, 线程总是并发而不是并行, 但仍需要使用互斥锁 (Mutual-Exclusion Locks, mutex) 来保证其线程安全. 因为 GIL 并不保证对数据结构的操作都是原子的. 同样由于 GIL, Python 会平均每个线程的占用处理器时长.

```python
counter.cnt += 1

# 实际 Python 执行了:
var = getattr(counter, 'cnt')
var2 = var + 1
setattr(counter, 'cnt', var2)

# 未加锁的数据竞争:
varA = getattr(counter, 'cnt') # from A
varB = getattr(counter, 'cnt') # from B
varB2 = varB + 1               # from B
setattr(counter, 'cnt', varB2) # from B
varA2 = varA + 1               # from A
setattr(counter, 'cnt', varA2) # from A
```

借助[上下文管理](开发工具/contextlib.md), 使用锁的方式很简单:

```python
class Counter:
	def __init__(self):
		self.lock = Lock()
		self.cnt = 0
	def plusone(self):
		with self.lock:
			self.cnt += 1
```

### Queue

协调多线程的方式之一是**流水线工作**, 即多线程组成*生成者-消费者管道*:

```
 worker1 -> worker2 -> worker3 -> ...
 `->` is queue with cache, like collections.deque
```

这种方式有几个棘手问题:
1. 如何监控管道是否有输入? 使用轮询结构, 此时后续线程持续饥饿, 占用 CPU 资源. 
```python
class Worker:
	def work(self):
		while True: # poll ++
			try:
				item = self.in_queue.get()
			except IndexError:
				sleep(0.01) # no work to do, tell cpu to come later
			else:
				result = self.func(item)
				self.out_queue.put(result)
				self.work_done += 1
``` 
2. 如何获取结果? 需要额外的结果队列 `done_queue`, 并轮询访问, 占用 CPU 资源.
3. 轮询如何退出? 捕获信号, 复杂.
4. 如何处理流水线阻塞? 线程处理任务有快有慢, 慢速线程的输入队列会不断堆积快速线程的输出, 直到耗尽系统资源. 需要限制缓存大小.

鉴于手动实现生成-消费者队列的复杂性, 推荐使用 python 标准库的 `Queue`

```python
from queue import Queue
queue = Queue()

def consumer():
	queue.get() # blocking until something is put()

thread = Thread(target=consumer)
thread.start()
queue.put(object()) 
thread.join()
```

Queue 可以限制队列的缓存上限, 输入数量超出上限时, 所有 `put()` 将阻塞. 调用 `task_done()` 帮助 Queue 标记任务完成. 当所有任务被标记完成后, Queue 才可以被 `join()`, 不再阻塞.
```python
queue = Queue(1) # buffer size of 1

def consumer():
	work = queue.get()  # 2
	# ...
	queue.task_done()   # 3 mark `work` as completed

queue.put(object())   # 1
queue.join()          # 4
```

借助 `Queue` 完成管道访问, 并支持手动调用 `close()->join()` 停止工作:

```python
class ClosableQueue(Queue):
	SENTINEL = object() # also ok to use None
	def close(self):
		self.put(self.SENTINEL)
	def __iter__(self):
		while True:
			item = self.get()
			try:
				if item is self.SENTINEL:
					return # signal: no more input
				yield item
			finally: # context management
				self.task_done()

class Worker:
	def work(self):
		for item in self.in_queue: # like coroutine
			result = self.func(item)
			self.out_queue.put(result)

...
done_queue.close()
done_queue.join()
```

## 协程

python 使用生成器的"暂停-恢复"特性来**模拟**协程, 开销极小. 协程依赖于当前线程, 由于 GIL 限制, 线程无法并行, 所以协程也不能**并行**执行. 使用协程目的是减少线程上下文切换的开销, 所以其实质是更便捷的同步工具.

```python
def minimize(): # coroutines
	current = yield # pause at each yield
	while True:
		value = yield current
		current = min(value, current)
it = minimize()
next(it) # prime the generator
print(it.send(10)) # resume coroutine by send()
print(it.send(4))
print(it.send(20))
print(it.send(-1))

>> 10
>> 4
>> 4
>> -1
```

- `yield` 将函数变为生成器函数, 每次执行到 yield 时会暂停, 并返回 yield 后的值.
- `yield from` py3.3 以上引入的特性, 简化调用另一个生成器的过程(不必使用循环), 当前生成器会暂停直到嵌套的生成器完成. 将另一个协程的控制权和结果直接传递出去.
- `next()`
- `send()` 向生成器发送数据, 作为阻塞 yield 的返回值, 同时启动 yield 的执行.
- `async def` 开启异步函数. py3.5以上引入的 [`asyncio` 库](asyncio.md), 实现真正的协程.
- `await` 用于挂起当前协程直到某异步操作完成.

### asyncio 

Python3.7 中, 官方对*协程*的支持. 

```python
import asyncio 

async def my_coros():
	pass
	
await my_coros()
```