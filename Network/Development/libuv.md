**Handles and Requests**:
- Handles: long-lived objects capable of performing certain operations while active.
- Requests: short-lived operations, over a handle or a standalone 
- Event Loop

异步任务被挂到内部线程池, 后台执行. 任务完成后, 将通知主线程串行执行回调函数.

## 参考

https://docs.libuv.org/en/v1.x/design.html