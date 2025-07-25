## IPv4

### IP 地址分类

开头为哈夫曼编码, 对 IP 进行唯一区分

|     | 网络地址位宽    | 主机地址位宽 | 固定前缀格式     |
| --- | --------------- | ------------ | -------- |
| A   | `8b`            | `24b`        | `0...`   |
| B   | `16b`           | `16b`        | `10...`  |
| C   | `24b`           | `8b`         | `110...` |
| D   | `32b`, 用于多播 |              | `1110...`   |
| E   | `32b`, 保留后用 |              | `1111...`         |

5 类地址分类不够灵活, 比较小的网络容易出现地址资源浪费. 现在逐渐被 *CIDR 策略 (Classless Inter-Domain Routing, 无分类域间路由选择)* 取代, 其允许任意长度的子网掩码, 用斜杠后数字来表示掩码的长度, 使网络的划分和分配更加灵活. 例如: `192.180.1.0/24`

#### 非单播特殊地址

| 网络段 | 主机段 | 作为源地址 | 作为目的地址 | 用途                   |
| ------ | ------ | ---------- | ------------ | ---------------------- |
| xxx       | 全 1   | 不可以     | 可以         | 广播地址               |
| 全 1   | 全 1   | 不可以     | 可以         | 本网络的广播地址       |
| 127    | xxx       | 可以       | 可以         | 本地回环测试           |
| 全 0   | 全 0   | 可以       | 不可以       | 默认路由, 本网络本主机 |
| 全 0   |  xxx      | 可以       | 不可以       | 本网络某主机           |
|    xxx    | 全 0   |       |              | 网络地址               |


#### 内网地址

- `10.0.0.0` -- `10.255.255.255` RFC 1918 定义的私有网络
- `172.16.0.0` -- `172.31.255.255` ..
- `192.168.0.0` -- `192.168.255.255` ..
- `100.64.0.0` -- `100.127.255.255` RFC 6598 定义的用于私有网络地址转换 (Carrier-Grade NAT, CGN) 的网段, 进一步提高 NAT 规模.

> 例子: `128.14.32.0/20`, 子网掩码为 `255.255.240.0`, 网络地址的前 20 位和给定的 `128.14.32.0` 保持一致,
>  网络范围为: `128.14.32.0 ~ 128.14.47.255`

#### 运营商级 NAT 共享地址

由于公网地址紧缺, 运营商 (ISP) 也无法获得新的公网 IP 地址. 为满足新用户入网需求, RFC6598 规定了 `100.64.0.0/10` 用作运行商级 NAT 共享地址 (Carrier-Grade NAT, CGN). CGN 地址只能用于 ISP 内部网络.

用于通过 ISP 访问互联网需要经过两次 NAT:
1. 内网网关 NAT, 由内网专用地址转为 CGN 地址.
2. 运营商网关 NAT, 由 CGN 地址转为 公网地址.

### IP 数据报

首部前 20 字节固定长, 其后长度可变.

```
  0                   1                   2                   3   
  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |  Ver  |  IHL  |Type of Service|          Total Length         |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |        Identification         |Flags|       Fragment Offset   |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |  Time to Live  |   Protocol   |       Header Checksum         |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |                        Source IP Address                      |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |                     Destination IP Address                    |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |                             Options             |   padding   |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |                             Data                              |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```

- Version: 版本号 (IPv4: 4, IPv6: 6)
- IHL (Internet Header Length):  IP 头长度, 单位 4 bytes.
- Type of Service: usually unused
- Total Length: 数据 + 头部总长度, 单位 bytes.
- Identification: 用于分片, 标识同一数据的分片
- Flags: `Reserved|DF|MF`, DF: don't fragment 不分片, MF: more fragment 后续还有改报文的分片.
- Fragment Offset: 分片原偏移, 单位 8 bytes
- Time to Live: 每转发一次, 减一.
- Protocol: 数据是哪个上层协议负责处理的, 比如 ICMP/OSPF/TCP/UDP 等.
- Options: 可扩展功能段

### MTU

*MTU (Maximum Transmission Unit, 最大传输单元)* 指协议在单个数据包中能传输的最大数据量. 以字节为单位, 数据量包括 IP 头和 MAC 头, 并且和实际的网络协议和技术有关. 

当单一数据包大小超过路径任一网络链路的 MTU 时, 其会被拆分为多个更小的片段来传输. 该过程称为*分片 (Fragmentation)*. 该过程会消耗网络性能.

以太网标准的 MTU 为 1500 字节, MAC 头为 14字节, IPv4 头为 20 字节, 实际的*载荷 (Payload)* 为 $1500-14-20=1466$ 字节.

## 路由表

```
目的地址     掩码  下一跳地址                转发接口
0.0.0.0     0    192.168.0.1 (gateway)   eth0      (default routing)
182.168.1.0 24   10.0.0.2                eth0
10.0.0.0    24   direct                  eth1
```

根据目的地址匹配路由表项的算法是*最长前缀算法*. 除了静态配置路由表, 动态维护和更新路由表的算法称为*路由选择协议 (Routing Protocol)*. 路由选择协议适合较大的网络, 目标是找出源点和目的点间的**最低路由开销路径**, 以减少整体网络负担.

|  [路由选择协议](路由选择协议.md)    | BGP | RIP | OSPF  |
| -------- | ---------------------------- | ---------------------------- | ------------------------------ |
| 算法     | 路径矢量算法 (PV)                 | 链路状态算法 (LS)                | 距离矢量算法 (DV)                  |
| 应用范围 | EGP, 互联网级别大型网络      | IGP, 小型自治系统内部        | IGP, 中型自治系统内部          |
| 收敛速度 |                              | 较慢                         | 较快                               |

## IPv6

ipv6 和 ipv4 是不兼容的, 由于侵犯了 ipv4 提供商 (国内 ISP 和云服务商这类赛博地主) 的利益, 导致从 ipv4 向 ipv6 过渡困难.



## DHCP

动态分配主机地址, 见 [DHCP](../ApplicationL4/DHCP.md).
