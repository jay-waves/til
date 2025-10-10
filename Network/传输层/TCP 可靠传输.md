
## 可靠传输原理

首先假定理想传输条件, 然后逐渐去除假定条件, 构造可以在不可靠信道上传输的单项数据通信协议.[^1]
- 信道不产生差错, 丢包或延迟.
- 不管发送速度多快, 接收方总来得及处理收到的数据.

### 停止等待协议

首先, 去掉假设条件 (2): 接收方总有足够的接受能力. 此时需要我们增加*流量控制机制*, 抑制发送端速度, 使接收端来得及处理. 

#### PDU 出错或丢失

#### ACK 出错或丢失

#### ACK 延迟

#### 停等协议的新年到利用率

### 滑动窗口协议

拥塞控制: 控制网络输入不超过网络负载

#### 回退 N 步的滑动窗口协议

### 选择重传协议

## 流量控制

## 拥塞控制

![|350](../../../attach/计算机网络_拥塞控制.avif)

![|500](../../../attach/Pasted%20image%2020230604173012.avif)

**慢启动算法**:

```
Initialize ssthesh = ?
Initialize cwnd <- 1 // 1 MSS
For (each segment ACKed):
	cwnd <- cwnd + 1
Until (loss event OR cwnd > ssthresh):
```

![|500](../../../attach/计算机网络_拥塞控制_自动机.avif)

算法流程:

```
cwnd = 1
loss_event = { 3ACK, TimeOut, None }
state = { SlowStart, CongestionAvoid }

Loop:
	send cwnd * TCP segments;
	If (state == SlowStart):
		For (each segmaent ACKed):
			cwnd <- cwnd + 1;
	Else If (state == CongestionAvoid):
		cwnd <- cwnd + 1;

	# state convert
	If (loss_event == 3ACK):
		ssthresh <- cwnd/2;
		cwnd <- ssthresh;
	Else If (loss_event == TimeOut):
		ssthresh <- cwnd/2;
		cwnd <- 1;
	Else If (cwnd > ssthresh):
		state <- CongestionAvoid
	Else:
		state <- state;
```

`MaxWnd = Min[rwnd, cwnd]`


[^1]: 谢希仁, *计算机网络*, 经典讲解方式, 这里不做修改.