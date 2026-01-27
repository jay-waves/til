`io_uring` 包含两个内核、用户态**共享**的循环队列：
* Submission Queue, SQ. 用户态写入 SQE （发起请求），内核态读取
* Completion Queue, CQ. 内核异步完成执行请求，结果写入 CQE ，用户态读取

通过 `liburing` 库使用 `io_uring`。