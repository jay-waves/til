
基于 发布/订阅 模型, Publisher 通过 Topic 发布数据, Subscriber 订阅 Topic 来获取数据. 通信解耦, 支持 点对点 / 组播 / 广播, 通信在同一*域 (Domain)* 下进行.

DDS 提供丰富的 QoS (Quality of Service) 策略:
- 数据持久性 (Durability)
- 可靠性 (Reliability)
- 优先级 (Latency Budget)
- 数据生存时间 (Lifespan)

