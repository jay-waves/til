读写锁 (Read-Write Lock) 思想是, 多线程同时*读数据*是线程安全的, 而*写数据*是不安全的, 若临界数据经常被读取而较少写入时, 使用普通锁是低效的. 

| state of rw_lock | shared access | exclusive access |
| ---------------- | ------- | --------- |
| free             | succeed | succeed   |
| shared           | succeed | wait      |
| exclusive        | wait    | wait          |

rw_lock 在设计上，通常优先保证**写操作优先**。即，当有写操作在等待锁时，会阻止新的读操作获取锁。直到旧读操作全部释放后，优先进行写操作。如果是读操作极端多的情况，则可以考虑用 [RCU](read-copy-update.md)。

## Go Mutex 

```go
type Mutex struct {
	key  int32 
	sema int32
}

func xadd(val *int32, delta int32) (new int32) {
	for {
		v := *val 
		if cas(val, v, v+delta) {
			return v + delta
		}
	}
}

func (m *Mutex) Lock() {
	if xadd(&m.key, 1) == 1 {
		return 
	}
	semacquire(&m.sema)
}

func (m *Mutex) Unlock() {
	if xadd(&m.key, -1) == 0 {
		return 
	}
	semarelease(&m.sema)
}
```

Go 这里不强制要求**谁申请、谁释放**。

在实际的实现中，goroutine 会先自旋一会，尝试忙等获取锁，超时后再休眠等待。这对于短临界区操作比较有效。

## Go RWMutex

```go
type RWMutex struct {
	w           Mutex; 
	writerSem   uint32 
	readerSem   uint32
	readerCount int32 
	readerWait  int32 // writer 等待时，用于统计活跃的 reader 数量
}

const rwmutexMaxReaders = 1 << 30
```

#### RWMutex.RLock && RUnlock 

`readerCount` 为负数时，意味着有 writer 在等待，此时新 reader 都要阻塞休眠。

```go
func (rw *RWMutex) RLock() {
	if atomic.AddInt32(&rw.readerCount, 1) < 0 {
		runtime_SemacquireMutex(&rw.readerSem, false, 0)
	}
}

func (rw *RWMutex) RUnlock() {
	if r := atomic.AddInt32(&rw.readerCount, -1); r < 0 {
		if atomic.AddInt32(&rw.readerWait, -1) == 0 {
			runtime_Semrelease(&rw.writerSem, false, 1)
		}
	}
}

```

#### RWMutex.Lock 

将 `readerCount` 反转，通知其他 reader：“当前有 writer 在等待锁”

```go
func (rw &RWMutex) Lock() {
	rw.w.Lock() 
	r := atomic.AddInt32(&rw.readerCount, -rwmutexMaxReaders) + rwmutexMaxReaders 
	
	if r != 0 && atomicc.AddInt32(&rw.readerWait, r) != 0 {
		runtime_SemacquireMutex(&rw.writerSem, false, 0)
	}
}
```

#### RWMutex

```go
func (rw *RWMutex) Unlock () {
	r := atomic.AddInt32(&rw.readerCount, rwmutexMaxReaders) 
	
	for i := 0; i < int(r); i++ {
		runtime_Semrelease(&rw.readerSem, false, 0)
	}
	
	rw.w.Unlock()
}
```

## RWLock 死锁

Reader 活跃时，请求 Writer 操作（内部可能有 Lock 方法），会导致循环依赖，死锁。

Reader 活跃，并且有 Writer 等待时，Reader 请求新 Reader 操作，会导致循环等待，死锁。

以上两种情况，都是由于 Mutex 重入导致的。