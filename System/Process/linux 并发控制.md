## 竞态发生场景

在 *对称多处理器 (SMP)* 场景下, 竞态 (Race Condition) 可能发生在 CPU0 进程和 CPU1 进程之间, CPU0 进程和 CPU1 中断之间, CPU0 中断和 CPU1 中断之间, CPU0 进程和 CPU0 中断之间. 对于 Linux 内核 (2.6 之后), 内核支持进程被高优先级进程打断, 称为内核抢占调度. 内核 2.6.35 版本以后, 不再支持单核中断的嵌套.

内核提供的互斥保护机制包括: *中断屏蔽, 原子操作, 自旋锁, 信号量, 互斥体*. 可参考 [进程同步与互斥](进程同步与互斥.md), 以及[硬件内存模型](../../HardWare/计算机组成/内存模型.md)

### 乱序执行问题

内核不喜欢 C `volatile` 关键字, 认为其约束力弱, 见 `Documentation/volatile-considered-harmful.txt`

屏障指令见 `Documentation/memory-barriers.txt` 和 `Documentation/io_ordering.txt`

处理器屏障类型:
- 内存屏障 (Memory Barrier). 保证内存操作按程序指定的顺序执行, 屏障前所有内存操作完成后, 再执行屏障后操作. 可细分为读 / 写屏障.
- 同步屏障 (Synchronizatoin Barrier). 不仅约束内存操作顺序, 还保证相关硬件状态 (缓存, 缓冲区) 同步完成.
- 指令屏障 (Instruction Barrier). 刷新指令流水线, 确保屏障后的指令基于最新的处理器状态.

内核 API:
- `barrier()` 用于防范编译乱序问题
- `mb()` 读写屏障, `rmb()` 读屏障, `wmb()` 写屏障.
- 外设屏障: `__iormb(), __iowmb()`

## 中断屏蔽

对于单核 CPU 而言, 进入临界区前屏蔽中断即可避免竞态. 由于 Linux 内核的进程调度也依赖中断来实现, 实际也能避免内核抢占进程导致的竞争状态. **中断屏蔽的原理是使 CPU 本身不响应中断**. 比如, SPARC 架构中, 使 CPU PSR 寄存器中 ET 位为 0, 即可在硬件层次上屏蔽中断.

```c
local_irq_disable();

// critical section

local_irq_enable();


local_irq_save(); // 保存当前 CPU 中断前状态 (PSR), 再禁止中断

// critical section

local_irq_resotre()
```

`local_` 前缀意味着, 这些调用只能禁止本 CPU 内部的中断, 不能解决 SMP 情景多处理器导致的竞态. 另外, 内核正确运行依赖于中断, 长时间屏蔽中断对系统是危险的. 因此, 独立使用 `irq_disable()` 不是推荐办法, 

## 内核原子操作

内核提供了对 *位和整型变量* 的原子操作接口, 这些接口也依赖于底层 CPU 功能.

```c
void atomic_set(atomic_t *v, int i);
atomic_t v = ATOMIC_INIT(0); 

atomic_read(atomic_t *v); // 返回原子量的值

void atomic_add(int i, atomic_t *v);
void atomic_sub(int i, atomic_t *v); 
void atomic_inc(atomic_t *v); // 自增
void atomic_dec(aotmic_t *v);

// 先计算, 然后测试是否为 0, 为 0 则返回 true.
int atomic_inc_and_test(atomic_t *v);
int atomic_dec_and_test(atomic_t *v);
int atomic_sub_and_test(int i, atomic_t *v);

// 先计算, 并返回新值
int atomic_add_return(int i, atomic_t *v);
```

位原子操作:

```c
// 操作 addr 地址上的第 nr 位, 置为 1
void set_bit(nr, void *addr);
// 置为 0
void clear_bit(nr, void *addr);
// 反转位
void change_bit(nr, void *addr);
// 测试位
test_bit(nr, void *addr);

// test_and_xxx_bit 先 test_bit 再执行操作 xxx_bit 
int test_and_set_bit(nr, void *addr);
```

使用原子变量实现信号量:

```c
static atomic_t xxx_available = ATOMIC_INIT(1);
static int xxx_open(struct inode *inode, struct file *filp)
{
	if (!atomic_dec_and_test(&xxx_available)) {
		atomic_inc(&xxx_available);
		return -EBUSY;
	}
	...
	return 0;
}

static int xxx_release(struct inode *inode, struct file *filp)
{
	atomic_inc(&xxx_available);
	return 0;
}
```

## 自旋锁


使用非自旋锁, 当前进程等待资源时, 会主动释放 CPU, 进入睡眠状态. 满足"让权等待"原则, 提高并发效率. 但对于较小较底层的临界区, 切换任务上下文的开销较大; 同时, 让出 CPU 等于让出缓存和 TLB, 当再次获取 CPU 时, 可能缓存已被污染, 需要重新缺页中断和读缓存.

```c
	wait(S) {
		while (S <= 0)
			sleep(0.1); // actively release CPU.
		S--;
	}
```

相反, 等待*自旋锁 (Spin Lock)* 时, 线程会持续*轮询* (忙等待, busy-waiting), 而不释放 CPU 资源. 自旋锁用于可抢占的调度策略中, 保证临界区不受其他 CPU 或本 CPU (的进程) 的抢占. 但不禁用中断 (指硬件中断), 不假设中断竞争.

```c
spinlock_t lock;
spin_lock_init(&lock); // 这是一个宏

spin_lock(&lock); // 获取自旋锁. 获取则返回, 否则等待.

spin_trylock(&lock); // 获取自旋锁. 获取则返回 true, 否则返回 false. 不阻塞等待.

spin_unlock(&lock); // 释放自旋锁
```

假设 CPU 单核心持有自旋锁进入临界区, 此时刻被硬件中断打断, 同时中断处理程序也试图获取同一个自旋锁. 此时会发生: 中断程序发现锁被占用, 自选等待; 实际持有锁的代码已因中断暂停, 释放了 CPU 使用权, 因此无法释放锁, 这导致了[死锁](进程同步与互斥.md).

即使禁用了中断, 中断下半区仍可能在自旋锁释放后立即被调度执行. 若下半区代码也试图访问同一临界区, 并试图获取同一自旋锁, 同样可能导致死锁和数据竞争.

因此要在多核环境下, 完整保护临界区, 需要同时: 使用自旋锁, 保护多核间并发; 禁用中断, 保护单一 CPU 上的中断上下文竞争.
- 结合 `local_irq_disable() / local_irq_enable()` --> `spin_lock_irq() / spin_unlock_irq()`
- 结合 `local_irq_save() / locak_irq_restore()` --> `spin_lock_irqsave() / spin_unlock_irqrestore()`
- 结合 `local_bh_disable() / local_bh_enable()` --> `spin_lock_bh() / spin_unlock_bh()`

...

**自旋锁假设临界区非常短, 锁很快被释放, 进程不释放 CPU 资源, 避免了上下文切换的开销.** 只有在占用锁的时间极短的情况下, 使用自旋锁才是合理的, 长时间使用自旋锁会导致 CPU 空耗, 降低系统性能. 递归获取自旋锁和中断上下文获取自旋锁都是危险的, 可能导致死锁. 

自旋锁锁定期间, 不能调用可能引起进程调度的函数. 如调用 `copy_form_user(), kmalloc(), ksleep()`  等阻塞函数 (这些函数需要睡眠, 释放 CPU, 和自旋锁相悖), 可能导致内核崩溃.

```c

...
```

### 读写自旋锁

### 顺序锁

顺序锁 (seq lock)

### 读-复制-更新

RCU, Read-Copy Update (Paul McKenney, 2001). 

## 信号量

[信号量 (Semaphore)](进程同步与互斥.md) 是经典的同步与互斥工具. 可以保护临界区, 当获取不到信号量时, 进程进入休眠等待, 而不是忙等待 (自旋锁). 在内核中, 信号量多用于同步, 内核更推荐直接用 mutex 作为临界区互斥手段.

```c
struct semaphore sem;

// 初始化值为 val
void sema_init(struct semaphore *sem, int val);

// 获取信号量 sem, 可能导致睡眠, 不再响应信号.
void down(struct semaphore* sem);

// 获取信号量, 可能导致睡眠, 但可以被信号打断. 可用于中断. 被打断时, 返回值非零.
int down_interruptible(struct semaphore *sem);
if (down_interruptible(&sem)
	return -ERESTARTSYS;
	
// 获取信号量, 如果能立刻获得, 就获得并返回零. 否则返回非零.
int down_trylock(struct semaphore *sem);

// 释放信号量, 唤醒等待者.
void up(struct semaphore *sem);
```

## 互斥体

Mutex



## 完成量

Completion

## 