Go 的调度模型是 M:N 式的, 有三个概念:
* G: goroutines, 需要执行的用户态 Go 程序任务
* M: machine, 指一个 OS 线程, 由操作系统调度控制
* P: processor, 逻辑 CPU 核心, 也指程序执行机会. 由 `GOMAXPROCS` 指定.

## Go CSP

GO 践行*消息传递风格 (CSP, Communicating Sequential Processes)* 的并发模型. **不通过共享内存来通信, 而是通过通信来共享内存**. 

## goroutine

> If a package `p` imports package `q`, the completion of `q`'s `init` functions
> happens before the start of any of `p`'s.  
> The completion of all `init` functions is synchronized before the start of 
> the function `main.main`.

Golang 使用 `go` 来开启 `goroutine` 协程, 由于协程是协作式的, 通常不需要锁而由用户直接控制切换, 被挂起即释放处理器, 用(语法上)类似同步的方式编写异步代码. 

|   go:         | 协程              | 线程       | 进程   |
| ---------- | ----------------- | ---------- | ------ |
| 地址空间   | 共享              | 共享       | 独立   |
| 堆         | 共享于线程        | 共享于进程 | 独立   |
| 栈         | 独立              | 独立       | 独立   |
| 锁需求     | 较少                  |            |        |
| 上下文切换 | 用户态            | 内核态     | 内核态 |
| CPU调度    | 协作式(暂停-恢复) | 抢占式     | 抢占式 |

> The `go` statement that starts a new goroutine is synchronized before 
> the start of the goroutine's execution

### Channel

详见 [Go 基础语法](编程语言/go/Go 语法.md).

### Locks

> For any `sync.Mutex` or `sync.RWMutex` variable `l` and $n<m$, 
> call _n_ of `l.Unlock()` is synchronized before call _m_ of `l.Lock()` returns.

go 中锁皆是不可重入 (ReentrantLock) 的, 即一个 goroutine 不能两次 Lock 同一锁.

```go
import . sync

type Locker interface {
	Lock()
	UnLock()
}
func (l *Mutex) Lock()
func (l *Mutex) UnLock()

// RWMutex 采用写优先策略, 读操作中, 如果有写请求, 不再处理读请求.
type RWMutex struct {
	w Mutex
	writerSem uint32
	readerSem uint32
	readerCount int32 // 有 writer 时, count 首位翻转为 1
	readerWait int32
}
func (rw *RWMutex) RLock()
func (rw *RWMutex) RUnLock()
func (rw *RWMutex) Lock()
func (rw *RWMutex) UnLock()
```

### Once

> The completion of a single call of `f()` from `once.Do(f)` is 
> synchronized before the return of any call of `once.Do(f)`.

```go
var once sync.Once
func a() {
	print("hello, world")
}
func b() {
	once.Do(a)
}
func main() {
	go b()
	go b()
}
```

### WaitGroup

```go
var wg sync.WaitGroup

for i := 0; i < 10; i++ {
    wg.Add(1) // cnt+1 before goroutine
    go func(i int) {
        defer wg.Done() // cnt-1 when finishied
        // do something
    }(i)
}

wg.Wait() // wait for all goroutines
```