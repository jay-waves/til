## Concept 

Sender 是一个异步延迟执行任务, 和一个Receiver 绑定后变为 op_state. op_state 调用 start 来实际执行异步任务, 结束后, 通过三种调用 value/error/stopped 来通知 receiver.

```cpp
concept scheduler:
	schedule(scheduler) -> sender;
	
concept sender:
	connect(sender, receiver) -> operation_state;
	
concept receiver:
	set_value(receiver, Values...) -> void;
	set_error(receiver, Error) -> void;
	set_stopped(receiver) -> void;
	
concept operation_state:
	start(operation_state) -> void;
```

[协程](coroutine.md)是一个状态机, 但不约束如何执行. execution 则明确地规定了任务如何*组合与调度*. execution 可以和 coroutine 紧密配合, 也属于结构化并发的一种. 

- set_value: 任务正常执行结束
- set_error: 出现错误, 将其传递给下游
- set_stopped: 提前取消 (中止). 通过 `stop_token` 实现, 需要 receiver 自定义响应: 轮询信号 / 注册回调 / 什么都不做. 

## Aaptors 

adaptor 是一种对 sender 的包装器, 用于改变 sender 的行为. 

```cpp
using ex = std::execution;

/*
	sender 在 scheduler 上执行
*/
ex::on(sched, s)
ex::starts_on(sched, s)
ex::continues_on

/*
	set_value 时调用 f(...)
*/
ex::then(s, f) -> s2;

/*
	调用 f(), 返回值
*/
ex::upon_error(s, f) -> v;
ex::upon_stopped(s, f) -> v;

/*
	调用 f(), 返回新 sender. 
*/
ex::let_value(s, f) -> s2;
ex::let_error(s, f) -> s2;
ex::let_stopped(s, f) -> s2; 

ex::bulk(n, f) -> f * n;
ex::bulk_chunked
ex::split 

/*
	并行启动多个 sender, 返回值元组. (树形并发结构)
*/
ex::when_all(s1, s2, ...)

/*
	sender 被取消时, 返回 nullopt. 用于与正常值区分.
*/
ex::stopped_as_optional
ex::stopped_as_error 

std::this_thread::sync_wait
```

```cpp
auto s = just(1)
		| then([](int x){ return x + 1; })
		| let_value([](int y){ return just(y*2); });
```

## Scheduler 

```cpp
class XxxScheduler {

friend auto tag_invoke(ex:schedule_t, const XxxScheduler& s) {
	return XxxSender{s};
}
};
```

## Reference 

https://www.youtube.com/watch?v=xLboNIf7BTg