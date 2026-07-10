## 网络防火墙

控制和管理网络访问, 将网络划分出信任区域, 充当网络边界. 通过定义的规则来允许或拒绝进出网络的流量, 通常基于网络地址, 端口号, 协议等报头信息对网络流量进行过滤.

防火墙衍生功能非常多: 会话管理, 安全区域, 安全策略, 路由交换, NAT, VPN, 入侵检测和防御 (报文攻击分析, DoS 防御, 网络扫描防御), 流量过滤, 监控和报告. 很多衍生安全设备也可以归于防火墙门下.

## 防火墙类型

### 1. 包过滤防火墙

Packet Filtering Firewall，也称静态包过滤。

主要检查报头信息，例如源/目的 IP、端口号、协议类型，并通过 ACL（Access Control List) 进行过滤。

### 2. 状态检测防火墙

Stateful Inspection Firewall，也称动态包过滤。

在包过滤基础上**增加会话状态跟踪**，能够判断数据包是否属于已建立、已批准的连接。

### 3. 代理防火墙

Proxy Firewall / Application Gateway。透明代理防火墙位于通信双方之间，代替客户端与外部服务通信。原本的一条端到端连接会被拆成两条连接：

```text
Client ── Firewall Proxy ── Server
```

常见形式包括：

* 电路级网关：工作在会话层，关注连接建立和维护，例如握手信息、序列号等，不检查应用层载荷。
* 应用层网关：工作在应用层，针对 HTTP、FTP、SMTP 等协议进行代理和内容过滤。
* 深度包检测（DPI,Deep Packet Inspection Firewall) 
* WAF Web Application Firewall ，云厂商在路由中提供，重点检测用于分析数据包内的 [SQL 注入, XSS 攻击等 WEB 安全威胁](../../sec/security-attack.md#1.3%20注入攻击).

> 防火墙的技术越复杂, 成本越高, 安全性越低. 反之亦然.

## 防火墙部署 / 接口模式

### 1. L3 路由模式 / NAT 模式

防火墙接口拥有 IP 地址，像路由器一样参与三层转发。
适用于需要路由、NAT、VPN、安全区域划分的场景。

```text
Client ── Gateway Firewall ── Internet
```

### 2. L2 透明模式

防火墙像交换机一样透明转发流量，不明显改变原有 IP 网络结构。
适用于不想大改网络拓扑、但希望在链路中插入安全检测的场景。

```text
Switch ── Transparent Firewall ── Router
```

### 3. L1 虚拟线缆模式

防火墙串联在线路中，不做路由，也不做传统二层桥接，逻辑上像一根带安全检测能力的线缆。
适用于对网络改动极小、只希望 inline 插入安全能力的场景。

```text
Device A ── Firewall as Cable ── Device B
```

### 4. TAP 旁路监听模式

防火墙或检测设备连接到交换机镜像端口，只观察流量，不参与转发。

适用于监控、审计、流量分析、入侵检测。它通常不能直接阻断流量。

```text
Network Link ── Switch
                  │
                  └── TAP / Mirror Port ── Sensor
```

## 防火墙功能

**安全区域与安全策略:**

*安全区域*: 防火墙以区域为对象分配安全策略, 区域分类包括 Trust Zone, Untrust Zone, DeMilitarized Zone (DMZ). 跨安全区域通信是**默认拒绝的 (Implicit Deny)**

*安全策略*: 维护访问控制列表 access list, 但是以 Zone 为单位进行控制. 内容安全策略包括: *反病毒*, *IPS*(Intrusion Prevention System)、*URL 过滤*、*DLP*(数据泄露防护) 等基于内容的安全机制.


### 提供 NAT

见 [nat](nat.md)

### 提供 VPN:

见 [vpn](vpn.md)


## 参考

[Guidelines on Firewalls and Firewall Policy. NIST SO 800-41 Rev1. 2009](https://csrc.nist.gov/pubs/sp/800/41/r1/final) 
