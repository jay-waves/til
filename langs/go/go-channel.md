
通道类型属于引用类型, 并且其本身是并发安全的. 通道用于传递数据, 在两个 goroutine 间通信. channel 完全可被当作线程安全队列, 实际上其内部也是用循环队列来实现的. 

声明一个通道: 
- `ch := make(chan int)` 默认不带缓冲区, 必须同步收发. 
- `ch := make(chan int, 100)` 设定缓冲区大小, 允许异步. 超出缓冲区时发送方会阻塞.

```go
ch <- v    // 把 v 复制到通道 ch, 原 v 仍可用.
v := <-ch  // 从 ch 移动出数据, ch 中数据被销毁.
close(ch) //关闭通道
```

缓冲区通道: The $kth$ receive on a channel with capacity $C$ is synchronized before the completion of the $(k+C)th$ send from that channel completes.

```go
var limit = make(chn int, 3)
func main() {
	for _,w := range work {
		go func(w func()){
			limit <- 1
			w()
			<-limit
		}(w)
	}
	select{}
}
```

单向通道: 主要用于参数约束

```go
type Notifier interface {
	SendInt(ch chan<- int)
}
```

无缓冲 Channel 行为：

|        | `nil` | open channel            | closed channel |
| ------ | ----- | ----------------------- | -------------- |
| recive | block | block and wait sender   | read 0         |
| send   | block | block and wait receiver | panic          |
| close  | panic | close                   | panic               |

有缓冲 Channel 行为：

|         | `nil` | empty | full  | not full & not empty | closed                      |
| ------- | ----- | ----- | ----- | -------------------- | --------------------------- |
| receive | block | block | read  | read                 | read 0 if no value residing.  |
| send    | block | write | block | write                | panic                            |
| close   | panic | close | close, retain unused | close, retain unused                | panic                            |


## 工作池

```go
type Job struct {
	ID int
}

type Result struct {
	JobID int 
	Value int 
}

func worker(id int, jobs <-chan Job, results chan<-Result, done chan<-struct{}) {
	defer func() {
		done <- struct{}{}
	}()
	
	for job := range jobs {
	
		...
		results <- Result {
			JobID: job.ID,
			Value: ...
		}
	}
}

{
	jobs := make(chan Job)
	results := make(chan Result)
	done := make(chan struct{})
	
	for i := 1; i <= 3; i++ {
		go worker(i, jobs, results, done)
	}
	
	go func() {
		defer close(jobs)
		
		for i := 1; i <= 10; i++ {
			jobs <- Job{ID: i}
		}
	}()
	
	go func() {
		for i := 0; i < 3; i++ {
			select {
			case <-done:
				...	
			case <-time.After(5 * time.Second):
				...
			}
			<-done
		}
		
		close(results)
	}()
	
	for result := range results {
		...
	}
}
```

## 信号通知

使用 `channel` 来代替 `Cond` 实现 wait/notify 模式

```go
{
	go func() {
	
	}()
	
	termChan := make(chan os.Signal)
	signal.Notify(termChan, syscall.SIGINT, syscall.SIGTERM)
	<-termChan
	
	doCleanup()
}
```

## 任务编排

#### fire-and-forget 

用 `WaitGroup` 实现即可

#### or-done 

有多个任务同时执行，任意一个执行完成即可通知。

```go
func or(channels ...<-chan any) <-chan any {
	switch len(channels) {
	case 0:
		return nil
	case 1:
		return channels[0]
	}
	
	orDone := make(chan interface{})
	go func() {
		defer close(orDone)
		
		switch len(channles) {
		case 2:
			select {
			case <-channels[0]:
			case <-channels[1]:
			}
		default:
			m := len(channes) / 2
			select {
			case <-or(channles[:m]...):
			case <-or(channles[m:]...):
			}
		}
	}()
	
	return orDone
}

{
	<-or(
		task1(),
		task2(),
		task3(),
		task4(),
	)
}
```

#### fan-in 

多个输入一个输出的模式。

```go
func fanIn[T any](chs ...<-chan T) <-chan T {
	out := make(chan T)
	done := make(chan struct{}, len(chs))
	
	for _, ch := range chs {
		go func(c <-chan T) {
			defer func() {
				done <- struct{}{}
			}()
			
			for v := range c {
				out <- v
			}
		}(ch)
	}
	
	go func() {
		for range chs {
			<-done
		}
		close(out)
	}()
	
	return out
}
```

#### pipeline 

```go
func Map[T any, R any](in <-chan T, fn func(T) R) <-chan R {
	out := make(chan R)
	
	go func() {
		defer close(out)
		
		for v := range in {
			out <- fn(v)
		}
	}()
	
	return out
}

func Filter[T any](in <-chan T, keep func(T) bool) <-chan T {
	out := make(chan T)
	
	go func() {
		defer close(out)
		
		for v := range in {
			if keep(v) {
				out <- v
			}
		}
	}()
	
	return out
}
```

如果下游不再读取数据，`out <- v` 可能会卡住，造成 goroutine 泄露。如果上游 panic 退出，而忘记关闭 `in` ，`<-in` 也可能卡住，由于没有引用，此时 GC 也不会介入。更好的写法：

```go
func Map[T any, R any](
	ctx context.Context,
	in <- chan T,
	fn func(T) R,
) <-chan R {
	out := make(chan R)
	
	go func() {
		defer close(out)
		
		for {
			select {
			case <-ctx.Done(): // 防止 in 卡住
				return 
				
			case v, ok := <-in:
				if !ok {
					return
				}
				
				select {
				case <- ctx.Done(): // 防止 out 卡住
					return 
				case out <- fn(v):
				}
			}
		}
	}()
	
	return out
}
```