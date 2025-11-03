
@lamport1978

## Happens-Before

How to Make a Multiprocessor Computer That Correctly Executes Multiprocess Programs. (Lamport, 1979) 引入了事件偏序关系 (sequential consistency):

> The customary approach to designing and proving the correctness of multiprocess algorithms for such a computer assumes that the following condition is satisfied: the result of any execution is the same as if the operations of all the processors were executed in some sequential order, and the operations of each individual processor appear in this sequence in the order specified by its program. A multiprocessor satisfying this condition will be called _sequentially consistent_.

Time, Clocks, and the Ordering of Events in a Distributed System. Lamport. 1978. 引入了基于偏序的竞争检查算法.

- **并发执行是受通信事件决定的偏序关系, 而程序无法观察无通信事件的并发事件顺序**
- 如果所有互斥访问可以被线性排序 -> 无数据竞争执行
- 如果存在互斥访问不能被排序 -> 可能存在数据竞争

![|300](../../../attach/Pasted%20image%2020240407153656.avif)

### Algorithm

事件偏序关系检测算法: 通过节点通信维护全局时钟.
```go
// assume N processes in system
const int N = ...;

func initialize_vector_clocks() {
    for each process P
        vector_clock[P] = int[N]
        for i from 0 to N-1
            vector_clock[P][i] = 0
}

// observer has no idea about order of local event in different threads.
func on_local_event(process P) {
    vector_clock[P][P] += 1
}

// observer can only infer order by communication event.
func on_send_message(process P, message M) {
    vector_clock[P][P] += 1
    M.vector_clock = vector_clock[P]
    send(M) // broadcast local clock
}

func on_receive_message(process P, message M) {
    vector_clock[P][P] += 1
    for i from 0 to N-1
        vector_clock[P][i] = max(vector_clock[P][i], M.vector_clock[i])
}

func compare_vector_clocks(clock A, clock B) {
    let A_before_B = true
    let B_before_A = true
    for i from 0 to N-1
        if A[i] > B[i]
            B_before_A = false
        if B[i] > A[i]
            A_before_B = false
    
    if A_before_B and not B_before_A
        report "A happens-before B"
    else if B_before_A and not A_before_B
        report "B happens-before A"
    else
        report "A and B are concurrent"
}
```

共享数据 `x` 竞争检测算法:
```go
// continue...
func is_concrr(clock A, clock B) {
	if compare_vector_clocks(A, B)=="A and B are concurrent"
		return true
	else
		return false
}

// During a write operation on variable x
func on_write(x, clock W) {
    if is_concrr(W, x.last_write_clock)
        report "write-write race"
        
    x.last_write_clock = W
    x.concurrent_read_clocks.clear()
}

// During a read operation on variable x
func on_read_start(x, clock R) {
    if is_concrr(R, x.last_write_clock)
        for read_clock in x.concurrent_read_clocks
            if is_concrr(R, read_clock) // observer more than one
                report "read-write race"
                break
                
    x.concurrent_read_clocks.add(R)
}

// When a read operation finishes
func on_read_end(x, clock R) {
    x.concurrent_read_clocks.remove(R)
}
```

- Slowdown > 50x
- Memory Overhead!
- Improved in [FastTrack](FastTrack.md)

## Vector Clocks

 Virtual time and global states of distributed systems -- Mattern 1998

...
