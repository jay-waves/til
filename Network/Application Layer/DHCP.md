动态主机设置协议 (Dynamic Host Configuration Protocol, DHCP) 用于动态分配 IP 地址的应用层协议, 定义于 [RFC 2131](https://datatracker.ietf.org/doc/html/rfc2131). 基于传输层的 [UDP](../Transport%20Layer/UDP.md) 协议, 客户端使用 68 端口, 服务端使用 67 端口.

DHCP 通常用局域网内的一组 DHCP 服务器来管理网络参数的分配, 这些参数包括 *IP 地址, 子网掩码, 网关和 DNS*. 

## 分配地址的三种方式

- 自动分配: 将 MAC 地址与分配的 IP 绑定, 第一次成功分配地址后, 就永远使用该地址. DHCP 前身 BOOTP 协议采用该方式.
- 动态分配: 每次断开连接后, DHCP 客户端要释放之前被分配的 IP 地址.
- 手动分配: HDCP 服务器管理员手动为客户端指定地址, 也可以直接在客户端上配置. 一些关键设备 (服务器, 打印机, 路由器等) 需要 IP 地址稳定, 会采取此方式.

自动分配和手动分配获得的都是**静态地址**, 动态分配的地址则有**时效性**.

## 工作流程

### 首次接入

首次接入时, 客户端在局域网内还没有地址, 所以源地址皆设置为 `0.0.0.0`. 

1. DHCP Discover: 设备首次联网, **广播 `DHCP DISCOVER` 信息, 寻找局域网内 DHCP 服务器**.
2. DHCP Offer: DHCP 服务器收到请求后, **广播回应 `DHCP OFFER` 消息**, 其中包含一个地址的租约, **并为用户保留该地址**直到超时. 任意数量服务器可以响应同一个 `DISCOVER` 请求, 但是一个客户网卡只能接受一个租约提供. 租约中包含: IP 地址, MAC 地址, 掩码, 租期, DHCP 服务器的 IP.
1. DHCP Request: 客户端从 `DHCP OFFER` 中挑选一个地址, 然后**广播** `DHCP REQUEST` 信息, 请求**租用**该地址. 当所有 DHCP 服务器收到 `REQUEST` 消息后, 会收回所有可能已经提供 (`OFFER`) 给客户但并没有被选中的租约, 将地址重新放回可用地址池中. 
2. DHCP Acknowledgement: DHCP 服务器确认该请求, 发送 `DHCP ACK` 消息, 并告知*租期 (Lease Time)*.
3. DHCP Decline: 使用期间, 如果客户端发现地址冲突, 将向 DHCP 服务器发送 `DHCP DECLINE` 消息.

> 频繁发起 `DHCP DISCOVER`, 造成可用地址池耗尽, 就是 DHCP Flood DoS 攻击.

### 重新登录

1. 客户端直接单播发送前一次 `DHCP REQUEST` 消息给目标 DHCP 服务器
2. DHCP 查看租约表, 首先尝试让客户端使用原来的网络参数. 
3. 如果此 IP 地址已经无法再分配, 如地址租约已经过期, DHCP 服务器会回应 `DHCP NACK` 消息.
4. 客户端如果收到 `DHCP NAK`, 则重新发送 `DHCP DISCOVER` 来重复首次接入流程 (此次发送可以不广播).
5. 客户端如果收到 `DHCP ACK`, 则说明地址续租成功, 继续使用该地址.

### 主动释放地址

1. 客户端准备关机或下线
2. 向 DHCP 服务器发送 `DHCP RELEASE` 消息, 主动释放租用的地址.

### 跨网段提供服务

DCHP 服务器可以跨网段提供服务, 但需要路由器进行中继

1. 客户端广播 `DHCP DISCOVER` 报文
2. 路由器配置了中继, 即知道另一个网段中 DHCP 服务器, 将该 `DISCOVER` 报文转发 (单播) 给该 DHCP 服务器.
3. DHCP 回应 `DHCP OFFER` 给路由器, 路由器再广播到客户端.
4. ... 后续类似