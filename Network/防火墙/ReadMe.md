## 网络防火墙

通过一系列设定的规则, 过滤网络流量 (数据包). [参考](https://zhuanlan.zhihu.com/p/461078581)

### 防火墙类型

根据 [Guidelines on Firewalls and Firewall Policy. NIST SO 800-41 Rev1. 2009](https://csrc.nist.gov/pubs/sp/800/41/r1/final) 划分, 防火墙主要有四大类:
1. 包过滤防火墙 (Packet Filtering Firewall):  检查报头信息 (网络地址, 端口号, 协议类型), 通过访问控制列表 (ACL) 进行静态过滤.
	- 也称*静态包过滤 (Static Packet Filtering)*, 工作在 OSI 网络层, 成本较低.
1. 状态检测防火墙 (Stateful Insepction Firewall): 记录和跟踪会话状态, 检测数据包是否来源于 "历史的, 已建立的, 已批准" 的会话连接, 区分新建连接与已建连接. 
	- 也称*动态包过滤 (Dynamic Packet Filtering)*, 工作在 OSI 传输层, 
1. 代理防火墙 (Proxy Firewall / Application Gateway): 位于应用层, [充当内部网络和外部通信的代理](Network/网络设备及拓扑.md#安全设备), 深度检测应用层协议 (HTTP, FTP) 的数据内容. 
	- *电路级网关 (Circuirt-Level Gateway)*, 工作于会话层, 重点关注会话建立和维护, 包括对握手信息和序列号合法性的各类检查. 会在内外网主机间建立透明代理 (和 NAT 不同, 原本一个连接变为两个), 由于工作在会话层, 不能检测应用层载荷 (因为 SSL 工作在表示层, 所以会话层无法读取其上信息).
	- *应用层网关 (Application-Level Gateway)*. 也是代理, 但工作在 OSI 应用层, 因此可以过滤整个应用层的数据载荷. 针对每个服务运行独立的代理, 并逐个检查过滤.
	- *深度包检测防火墙 (Deep Packet Insepction, DPI)*, 分析数据包载荷.
	- *应用防火墙 (Web Application Firewall, WAF)*, 用于分析数据包内的 [SQL 注入, XSS 攻击等 WEB 安全威胁](Security/安全攻击.md#1.3%20注入攻击).
2. 混合型防火墙 (Hybrid Firewall): 综合前三种防火墙功能, 并且包括一些入侵检测防御 ([IDS/IPS](Network/防火墙/IDPS.md)) 和应用控制 (恶意软件, 病毒木马, 垃圾邮件, DDoS) 功能.
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

### 防火墙接口模式

- L3: NAT 模式, 类似路由器接口, 拥有 IP. 可静态配置 IP, 也可通过 DHCP 动态获取.
- L2: 透传(透明)模式, 类似交换机. 使用 VLAN 分配 IP.
- L1: 虚拟线缆, 将两个接口一个作为输入一个作为输出.
- TAP: 与交换机镜像端口连接的模式. 仅能检测, 无法阻止.

L1-3 是串联在网路上, 而 TAP 是旁挂在交换机上.

### 防火墙功能

- *会话管理*: 防范 Dos (Denial of Service) 攻击, 控制负载.
- *报文结构解析*: 防范非法报文.

**安全区域与安全策略:**

- *安全区域*: 设置 Trust Zone, Untrust Zone, DeMilitarized Zone (DMZ). 跨安全区域通信是默认拒绝的 (Implicit Deny)

- *安全策略*: 维护访问控制列表 access list, 但是以 Zone 为单位进行控制. 内容安全策略包括: *反病毒*, *IPS*(Intrusion Prevention System)、*URL 过滤*、*DLP*(数据泄露防护) 等基于内容的安全机制.

#### 提供 NAT

见 [NAT](NAT.md)

#### 提供 VPN:

见 [VPN](../VPN/VPN.md)


