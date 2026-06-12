NAT (Network Address Translator), 将私有 IP 转化为公网 IP. 运行 NAT 的路由器或防火墙叫做网关 (Gateway), 大型网络中路由器资源有限, 常将 NAT 置于防火墙上. 通过 NAT 技术, 多个子网 IP 可以共享少数公网 IP, 缓解了 IPv4 地址枯竭的问题, 同时隐藏内网网络拓扑, 增强了一定安全性.

NAT 有三种常用技术:
- SNAT, 静态地址转换, 此时内网地址和外网地址是一一对应关系, 存在静态转换表.
- DNAT, 动态地址转换, 此时内外网地址是多对一关系, 内网主机访问外网时, 从网关处申请一个外网地址, 记录在动态转换表中. 缺点是外网地址耗尽时, NAT 将拒绝服务.
- PAT, 端口地址转换, 当 DNAT 分配外网地址紧张时, 多个内网地址可以同时共享一个外网地址, 通过*端口号*区分报文对应的内网地址. 

### NAT 出站通信

内网主机访问外网, 称为**出站通信**. NAT 负责实现内网地址和外网地址之间的自动转换, PAT/DNAT 需要为每个请求创建一个动态映射条目, 维护出站连接的会话状态, 消耗资源更多.

```
PC1:
send packet:    | src_addr: 192.168.1.155 | src_port: 300 | packet_data |
  |
  v
gateway:
record in NAT table: | 192.168.1.155:300 <-> 210.10.20.20:14000 |
send packet:    | src_addr: 210.10.20.20 | src_port: 14000 | packet_data |
  |
  v
internet server:
receive packet: | src_addr: 210.10.20.20 | src_port: 14000 | packet_data | 
```

### NAT 入站通信

外网主机访问内网中服务器, 称为**入站通信**, NAT 负责将外部网关地址转换为服务器内网地址. 映射规则一般是提前配置的, 即 SNAT, 无须建立动态映射 (但也需要记录, 称为*连接追踪表 CTT*), 消耗资源少. 

在严格 NAT 模式下, 用户动态地入站通信难以实现, 该技术称为[nat-traversal](nat-traversal.md).

```
external PC:
send packet:    | dest_addr: 221.12.1.7 | dest_port: 80 | packet_data |
  |
  v
gateway: 221.12.1.7
NAT table: | 221.12.1.7:80 <-> 192.168.1.18:80 |
send packet:    | dest_addr: 192.168.1.18 | dest_port: 80 | packet_data |
  |
  v
internal web server:
receive packet: | dest_addr: 192.168.1.18 | dest_port: 80 | packet_data | 
```

PAT 地址转换表, 假设网关只有一个外部地址 `221.12.1.7`:

| 内网地址     | 内部端口 | 目的地址     | 目的端口 | NAT 端口 | 协议 |
| ------------ | -------- | ------------ | -------- | -------- | ---- |
| 192.168.10.1 | 300      | 210.10.20.20 | 80       | 14000    | TCP  |
| 192.168.10.1 | 301      | 210.10.20.20 | 21       | 14001    | TCP  |
| 192.168.10.3 | 1275     | 207.21.1.5   | 80       | 14003    | TCP     |

> 同时进行出站和入站的通信的网络模式称为点对点连接; 入站为主的服务网络, 称为传统 C/S (B/S) 网络通信模式.

### NAT 网络穿透

* 对端 A\B 主动向公网 STUN Server，Server 将 A\B 公网端点分别交换给对方
*  A\B 尝试直接访问对方的公网端点，如果对方 NAT 类型比较宽松，就可以直接 P2P 通信（国内一般不行）
* P2P 失败后，流量需要通过 TURN Server 中继

```
                 ┌─────────────── INTERNET ────────────────┐
                 │                                         │
                 │       ┌──────┐              ┌──────┐    │
                 │       │ STUN │              │ TURN │    │
                 │       └──▲───┘              └──▲───┘    │
                 └──────────┼────────────────────═╪═───────┘
                            │                     ║
                            │ map endpoint        ║ relay if P2P fails
                            │                     ║
┌──────────┐           ╔════╧════╗  · · · ·  ╔════╧════╗           ┌──────────┐
│ Client A │──────────▶║ NAT  A  ║·· punch ··║ NAT  B  ║◀──────────│ Peer B   │
│ private  │ outbound  ╚════╤════╝  attempt  ╚════╤════╝  outbound │ private  │
└──────────┘                ▲                     ▲                └──────────┘
                            └────── P2P if NAT mappings align ─────┘
```