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
```

### RWLock 

详见 [rwlock](../../algo/concurrency/rwlock.md)

### Once

多次对同一函数 `f()` 调用 `Once.Do(f)` ，只有第一次会被执行；其他次调用时，会阻塞直到第一次 `f()` 被执行完。

```go
// sync 
type Once struct {
	done uint32
	m    Mutex 
}

func (o *Once) Do(f func()) {
	if atomic.LoadUint32(&o.done) == 0 {
		o.doSlow(f)
	}
}

func (o *Once) doSlow(f func()) {
	o.m.Lock()
	defer o.m.Unlock() 
	
	if o.done == 0 {
		defer atomi.StoreUint32(&o.done, 1)
		f()
	}
}
```

注意，`f()` 中不能调用同一个 `sync.Once()` ，否则会递归死锁。另外，如果 `f()` 执行失败（返回 Err 或者 panic），Once 仍会认为已经执行完毕，不会再次执行。

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

### Cond 

类似条件变量，详见 [c++/condition_variable](../cpp/concurrency/condition_variable.md)。实际上，`Cond` 使用不多，简单场景多用 `WaitGroup`，而复杂场景会用 `Go Channel`。

```go

func NewCond(l Locker) *Cond 

func(c *Cond) Broadcast()

func(c *Cond) Signal() 

func(c *Cond) Wait() // need c.L
```

### Pool 

对象池化。仅适用于对象高频初始化和失效的情景。
* `Pool` 作为缓存并不可靠，因为 Go GC 会无通知地回收对象。
* 对于很小的对象，不如不用缓存
* 对于很大的对象（比如容量变得很大的切片），再放回 Pool 会浪费内存。由于 Go GC 清理的不确定性，这部分内存可能迟迟无法释放。

```go
var bufPool = sync.Pool{
	New: func() any {
		b := make([]byte, 0, 1024)
		return &b
	},
}

func work() {
	bp := bufPool.Get().(*[]bytes)
	b := (*bp)[:0]
	
	// use b...
	
	*bp = b
	bufPool.Put(bp)
}

```


### Context 

同时管理协程的取消（Cancel）、超时（Deadline）、值传递（Value）。是一种级联（链式）结构，事件传播非常自然。但是不建议滥用，因为可能让生命周期不明确或资源滞留（泄露）

```go
type Context interface {
	Deadline() (deadlin time.Time, ok bool) 
	Done() <- chan struct{}            
	Err() error                         // Context 被取消时，返回原因，否则返回 nil
	Value(key interface{}) interface{}  // 沿着 Context 链向上查找 key 对应值
}
```

构造函数（工厂）：

```go
ctx := context.Background()  // 空 
ctx, cancel = context.WithCancel(ctx) 
defer cancel()  // 记得使用后调用 cancal 

ctx, cancel = context.WithTimeout(ctx, 2 * time.Second)
defer cancel()

ctx = contxt.WithValue(ctx, "UserID", 42) // 放一些配置或不可变值
```

`WithCancel` 构造如下结构，`Done()` 返回后，取消会沿着子 Context 链传播。用于手动取消。
```go
// context.WithCancel return:
type cancelCtx struct {
	Context                        // parent context 
	done      chan struct{}
	mu        sync.Mutex
	err       error
	children  map[canceler]struct{} // children contex
}
```

`WithValue` 构造如下结构，`Value()` 先检查自身 `key` 如果不匹配就沿着父 Context 链向上查找。

```go
type valueCtx struct {
	Context   // parent 
	key, val any
}
```

`WithTimeout, WithDeadline` 返回如下结构，其中 `WithTimeout` 就是用 `time.Now() + duration` 来调用 `WithDeadline`。

```go
type timerCtx struct {
	cancelCtx  // 组合 cancelCtx 功能
	deadline time.Time 
	timer    *time.Timer 
}
```

用 `Context` 取消 goroutine：

```go
ctx, cancel := context.WithCancel(context.Background())
defer cancel()

go func() {
	for {
		select {
		case <- ctx.Done():
			return 
		default:
			// do something
		}
	}
}()

// cancel() here 
```