MQTT (Message Queuing Telemetry Transport) 基于 发布/订阅 模型的轻量传输协议. 用于物联网 (IoT), 有低带宽和高可靠性.

- 支持 发布/订阅 模型. 
- 报头最少仅有 2 字节.
- 客户端通过 Broker (消息代理服务器) 中转各类 Topic 数据. 
- 基于 TCP/IP, 但也可以通过 WebSocket. 

### QoS 

- QoS 0: 最多一次, 可能丢失
- Qos 1: 至少一次, 可能重复 
- QoS 2: 仅一次, 确保消息不丢不重复 

### 工作流程 

1. 设备 (客户端) 与 Broker 建立 TCP 链接.
2. 客户端订阅或发布某些 Topic .
3. Last Will & Testament (LWT): 客户端异常断开时, Broker 自动发布遗嘱消息.