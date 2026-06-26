## 核心概念

* `tokio::task`  任务
	* `tokio::task::spawn` 向运行时添加新异步任务
	* `tokio::task::spawn_local` 不可被其他工作线程窃取的任务
	* `tokio::task::spwan_blocking` 向运行时添加同步任务，该任务会有独立的同步线程执行，可能阻塞。
	* `tokio::task::yield_now` 立即放弃 CPU，进入调度
* `tokio::sync` 同步原语
	* channels: `onshot, mpsc, broadcast, watch` 分别对应 一次性一对一、扇入、多对多、扇出 消息通道
	* non-blocking `Mutex` `RwLock` `Barrier` `Notify`
* `tokio::time` non-blocking timer, timeout, sleep ...
* `tokio::runtime` 
* `tokio::io`  
	* I/O Primitives: `AsyncRead, AsyncWrite, AsyncBufRead` 
	* `tokio::net`
	* `tokio::fs`
	* `tokio::signal`
	* `tokio::process` 

## tokio runtime 

`tokio::runtime` 是一个用户态 Pool Executor，`.await` 就是反复 `Poll(Future)` 直到得到 `Ready`。`tokio::runtime` 的调度单位是 `tokio::task`

* `spawn()` 不阻塞当前主线程
* `block_on()` 阻塞当前主线程

```rust
use tokio;

fn main() {
	tokio::runtime::Builder::new_multi_thread()
		.worker_threads(8)
		.enable_io()
		.enable_time()
		.build()
		.unwrap()
		.block_on(async { ... });
}

// 等价语法糖
#[tokio::main(worker_threads = 8))]
fn main() {
	
}
```

### async task 

`tokio::task::spawn` 是所有协程托管的起点，输入一个 Future（通常是 `async ...`），返回 `JoinHandle`。`JointHandle<T> = Future<Result<T, JointError>>`

`tokio::task::spawn` 要求的 Future 需要满足 `Send + 'statc`，也就是说：必须线程安全，因为 tokio runtime 可以跑在多个工作线程间；必须不能借用外部栈。

```rust
let listener = TcpListener::bind("127.0.0.1:8080").await?;

loop {
	let (mut socket, _) = listener.accept().await?;
	
	tokio::spawn(async move {
		let mut buf = [0; 1024];
		
		loop {
			let n = match socket.read(&mut buf).await {
				Ok(0)  => return, 
				Ok(n)  => n,
				Err(e) => {
					...
					return;
				}
				
				if let Err(e) = socket.write_all(&buf[0..n]).await {
					...
					return;
				}
			};
		}
	});
}
```

协程退出路径：
* 当协程内崩溃时，`JoinHandle` 会返回 `JoinError`，用 `JoinHandle.is_panic()` 判断
* 当协程主动取消时，用 `JoinHandle.is_cancelled()` 判断
* 当协程自行执行完毕后，用 `JoinHandle.is_finished()` 判断
* 调用 `JoinHandle::abort` 时，协程会在下一次 `.await` 调用点退出
* 直接从 `#[tokio::main]` 退出会立即终止所有协程

### blocking task 

一些可能阻塞的任务，`tokio` 允许单开一个阻塞线程来跑。

```rust
tokio::task::spawn_blocking( || {
	heavy_cpu_work_xxx();
});
```

### structured async task 

`join!` 类似 `when_all`，同时并发多个 `Poll(Future)`。

```rust 
// inside on task
tokio::join!(
	task_a(),
	task_b(),
	task_c(),
);

// return when any error (fast fail)
tokio::try_join!(
	a(),
	b(),
	c(),
);
```

`select!` 类似 [Go select](go/go-concurrency.md)] 关键字。同时 Poll 所有分支，并进入第一个完成的分支，多个完成的分支会被公平（随机）选择，其他则被丢弃。

```rust
// inside one task 
let winner = tokio::select! {
	value = op1() => {
		...
	}
	
	value = op2() => {
		...
	}
};
```

`JointSet` 可以组合多个 `tokio::task`，可以由多个工作线程执行，因此有机会并行。**注意，同一个 `tokio::task`在任意时刻只会被一个实际线程 Poll ，因此 `tokio::runtime` 的调度单位是 `tokio::task` ，而不是其内部的 Future**。

## Sync 

### channel 

`oneshot`  只提供一个方法：

```rust
pub fn channel<T>() -> (Sender<T>, Receiver<T>)

```

```rust
let (tx, rx) = oneshot::channel();

use tokio::sync::oneshot;

#[tokio::main]
async fn main() -> Result<(), &'static str> {
    let (tx, rx) = oneshot::channel();

    tokio::spawn(async move {
        let _ = tx.send(do_work());
    });

    let result = rx
        .await
        .map_err(|_| "channel closed")??;

    println!("success: {result}");
    Ok(())
}

fn do_work() -> Result<i32, &'static str> {
    Ok(42)
}
```


## IO