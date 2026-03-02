## RabbitMQ

RabbitMQ 保证**至少一次投递**的语义，因而所有消息都需要消费者确认。

概念：
* Producer 
* Exchange （交换机）：根据路由规则，投递到对应队列
* Queue
* Consumer 
* Binding：Exchange 与 Queue 间的实际规则

### 使用模式

#### Work Queue 

多个消费者竞争同一个队列

#### 发布订阅

使用 Fanout 类型 Exchange，给每个消费者分发同一个消息

#### 路由分发

Direct Exchange 精确分发；或 Topic Exchange 模糊匹配分发

#### 延迟队列

#### 死信队列
