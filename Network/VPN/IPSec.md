IPSec (Internet Protocol Security) 工作于网络层, 通过认证和加密保护 IP 报文. IPSec  常用于搭建企业间或远程访问的 VPN.

|                         | IPSec                         | TLS/SSL                             |
| ----------------------- | ----------------------------- | ----------------------------------- |
| 层次                    | 网络层                        | 传输层                              |
| 用途                    | 网关间加密, 端到端加密        | 应用层加密, HTTPS/电子邮件/即时通信 |
| 身份验证 (隧道建立方式) | IKE 协议                      | 数字整数                            |
| 数据完整性              | AH                            | HMAC                                |
| NAT 穿透                | 部分支持                      | 支持                                |
| 复杂性                  | 配置复杂, 需要客户端/网关软件 | 相对简单, 广泛封装于浏览器和应用    |
| 会话恢复                | 需要重新协商                  | 支持会话恢复 (Session Resumption)   |
| 流量保密性              | 隧道模式可加密 IP 头          | 不加密 IP 头, 也不加密 TCP 头                                    |

## 协议构成

IPSec 主要由**加密协议 (ESP, Encapsulating Security Payload)** 和**认证协议 (AH, )** 构成. Auth 协议负责认证数据, 即确保数据的真实性和完整性.

VPN-IPsec 建立过程:
```
PC A -> GateWay A   --- ipsec ---> GateWay B -> PC B
```

1. PC A 将报文发送给 网关 A (也是 VPN 设备)
2. 网关 A 加密报文, 添加 ESP 头部, 用公网发送给网关 B.
3. 网关 B 验证密文, 检查 ESP 和 AH 头部合规性.
4. 网关 B 取出冗余, 还原原始报文, 发送给 PC B

## 传输模式

Transport Mode. 仅对 IP 数据包的有效载荷部分 (Payload) 进行加密保护, IP 头不变. 主要用于端到端安全通信.

```
IP Header | Payload Data

|
v

IP Header | ESP Header | Payload Data | ESP Trailer | ESP Auth
```

- ESP Trailer 指填充信息和填充长度.

### 隧道模式

Tunnel Mode. 对整个 IP 数据报进行封装加密. 添加一个新 IP 头, 主要用于网关间通信.

```
IP Header | Payload Data

|
v

New IP Header | ESP Header | IP Header + Payload | ESP TRailer | ESP Auth 
```