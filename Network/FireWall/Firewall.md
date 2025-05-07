## 网络防火墙

控制和管理网络访问, 将网络划分出信任区域, 充当网络边界. 通过定义的规则来允许或拒绝进出网络的流量, 通常基于网络地址, 端口号, 协议等报头信息对网络流量进行过滤.

防火墙衍生功能非常多: 会话管理, 安全区域, 安全策略, 路由交换, NAT, VPN, 入侵检测和防御 (报文攻击分析, DoS 防御, 网络扫描防御), 流量过滤, 监控和报告. 很多衍生安全设备也可以归于防火墙门下.

### 防火墙类型

根据 [Guidelines on Firewalls and Firewall Policy. NIST SO 800-41 Rev1. 2009](https://csrc.nist.gov/pubs/sp/800/41/r1/final) 划分, 防火墙主要有四大类:
1. 包过滤防火墙 (Packet Filtering Firewall):  检查报头信息 (网络地址, 端口号, 协议类型), 通过访问控制列表 (ACL) 进行静态过滤.
	- 也称*静态包过滤 (Static Packet Filtering)*, 工作在 OSI 网络层, 成本较低.
1. 状态检测防火墙 (Stateful Insepction Firewall): 记录和跟踪会话状态, 检测数据包是否来源于 "历史的, 已建立的, 已批准" 的会话连接, 区分新建连接与已建连接. 
	- 也称*动态包过滤 (Dynamic Packet Filtering)*, 工作在 OSI 传输层和网络层. 
1. 代理防火墙 (Proxy Firewall / Application Gateway): 位于应用层, [充当内部网络和外部通信的代理](Network/网络设备及拓扑.md#安全设备), 深度检测应用层协议 (HTTP, FTP) 的数据内容. 
	- *电路级网关 (Circuirt-Level Gateway)*, 工作于会话层, 重点关注会话建立和维护, 包括对握手信息和序列号合法性的各类检查. 会在内外网主机间建立透明代理 (和 NAT 不同, 原本一个连接变为两个), 由于工作在会话层, 不能检测应用层载荷 (因为 SSL 工作在表示层, 所以会话层无法读取其上信息).
	- *应用层网关 (Application-Level Gateway)*. 也是代理, 但工作在 OSI 应用层, 因此可以过滤整个应用层的数据载荷. 针对每个服务运行独立的代理, 并逐个检查过滤.
	- *深度包检测防火墙 (Deep Packet Insepction, DPI)*, 分析数据包载荷.
	- *应用防火墙 (Web Application Firewall, WAF)*, 用于分析数据包内的 [SQL 注入, XSS 攻击等 WEB 安全威胁](Security/安全攻击.md#1.3%20注入攻击).
2. 混合型防火墙 (Hybrid Firewall): 综合前三种防火墙功能, 并且包括一些入侵检测防御 ([IDS/IPS](IDPS.md)) 和应用控制 (恶意软件, 病毒木马, 垃圾邮件, DDoS) 功能.
	- *下一代防火墙 (Next-Generation Firewall, NGFW)*
	- *统一威胁管理 (Unified Threat Management, UTM)*, 类似大杂烩.

按时间发展顺序: 
- 第一代: Cisco Inc. 1988. 路由器与静态包过滤防火墙.
- 第二代: AT&T Bell Labs. 1990. 电路级网络防火墙.
- 第三代: Purdue University. Bell Labs. 应用级网关防火墙.
- 第四代: USC, Checkpoint Inc. 1994. 动态包过滤防火墙.
- 第五代: NAI Inc. 1996. 代理防火墙.
- 最新: NGFM, UTM, WAF...

> 防火墙的技术越复杂, 成本越高, 安全性越低. 反之亦然.

### 访问控制列表

访问控制列表 (Access-List, ACL), Cisco 最初的路由器有访问控制控制功能, 也被视为内嵌包过滤防护墙的雏形. 访问控制表列表的表项包括三个元素: 对象, 行为和选型 (object, action and option).

如路由器访问控制列表中, 对象可以是 "地址, 协议, 端口, 历史会话" 等, 行为可以是*允许或拒绝 (permit & deny)* 二选一, 选项有 "记录日志, 表项有效时间" 等操作. 

```
# 允许 172.16.1.1 服务器向 10.1.1.2 的客户端提供 telnet 服务.
access-list 101 permit tcp host 10.1.1.2 host 172.16.1.1 eq telnet
```

### 防火墙接口模式

- L3: NAT 模式, 类似路由器接口, 拥有 IP. 开启路由, NAT, VPN 的防火墙, 必须使用该模式.
- L2: 透传(透明)模式, 类似交换机, 对报文进行交接. 在各个 VLAN 内拥有 IP 地址.
- L1: 虚拟线缆模式, 串联在一条线路上, 而不进行路由和桥接.
- TAP: 与交换机镜像端口连接的模式. 仅能检测, 无法阻止.

L1-3 是串联在网路上, 而 TAP 是旁挂在交换机上.

### 防火墙功能

**安全区域与安全策略:**

- *安全区域*: 防火墙以区域为对象分配安全策略, 区域分类包括 Trust Zone, Untrust Zone, DeMilitarized Zone (DMZ). 跨安全区域通信是**默认拒绝的 (Implicit Deny)**

- *安全策略*: 维护访问控制列表 access list, 但是以 Zone 为单位进行控制. 内容安全策略包括: *反病毒*, *IPS*(Intrusion Prevention System)、*URL 过滤*、*DLP*(数据泄露防护) 等基于内容的安全机制.

#### 提供 NAT

见 [NAT](NAT.md)

#### 提供 VPN:

见 [VPN](../VPN/VPN.md)


