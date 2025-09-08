## Concept 

- Sender: connect(sender, receiver) -> op_state
- Receiver: set_value, set_error, set_stopped
- OperationState : op_state.start()
- Schduler:  schedule(sched)->sender 

## Aaptors 

```cpp
using ex = std::execution;

execution::starts_on

execution::continues_on

ex::on

ex::then

ex::upon_error
ex::upon_stopped 

ex::let_value 
ex::let_error 
ex::let_stopped 

ex::bulk 
ex::bulk_chunked
ex::split 

ex::when_all
ex::stopped_as_optional
ex::stopped_as_error 

std::this_thread::sync_wait
```

## Reference 

https://www.youtube.com/watch?v=xLboNIf7BTg