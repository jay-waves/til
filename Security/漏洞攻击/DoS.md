DDos (Distributed Denial of Sevice) 通过僵尸网络的多个跳板, 对服务器发起攻击.

**攻击类型:**
- Syn Flood: 发送大量 TCP SYN 报文, 使服务器无法建立 tcp 连接.
- ICMP Flood: 发送大量 ICMP echo request.
- UDP/IP Flood: 发送大量 udp, 占用内存.
- Land: 发送目的地址和源地址相同的报文, 使服务器不断向自己发送数据.
- Tear Drop: 发送伪造的, 含有假 offset 的 tcp 报文.
- Ping of Death: 超过 IP 最大长度 65535 的 ping.
- Smurf: 广播源地址为攻击目标的 ICMP echo request, 使攻击目标收到大量 ICMP echo reply.
- Fraggle: 类似 Smurf, 但使用 UDP.
- Connection Flood: 开启大量连接, 不释放.
- Reload: 不断请求网页重载 (浏览器 F5).

**防御 Dos:**

限制异常高速的通信流量.