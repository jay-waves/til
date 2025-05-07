一个家庭网络示例: 光纤入户, 光猫为 ZXHN F650 4.0, 路由器为 小米 AX1800. 

```
          |---------|          |-------------|            |---------|
~~~FTTH~~~|ZXHM F650| -------- |XiaoMi AX1800| (((WIFI))) |Computers|
          |---------|          |-------------|            |---------|
          192.168.1.1          192.168.31.1               192.168.31.xxx
```

由于中国 IPv4 公网地址较少, 运营商通常只给企业大型网络分配固定公网地址, 动态给家庭企业分配 IP 地址, 或者使用 CGNAT (Carrier-Grade NAT) 技术让区域内多个用户共享一个公网地址. 入户后, 家庭内部还需要 NAT 一次, 实际连接性已经很差.


## 内网穿透

rathole 协议, websocket 协议, webRTC

互联网用户间通信都是入站通信, 两用户需要借助企业的服务器来实现通信, 并不直接点对点通信. 由于用户处于各自内网中, 要实现直接点对点, 需要用到内网穿透技术.

- STUN (Seesion Traversal Utilities for NAT): UDP 通信, 需要借助一个公共 STUN 服务器交换信息, 对于严格 NAT 模式 (对称 NAT), 无法穿透.
- TURN (Traversal Using Relays around NAT) 协议, 是 STUN 失败时的替代方案, TURN 服务器模拟企业服务器的双入站通信方式, 来为双方提供点对点通信. 
- UDP Hole Punching (打孔): ..., 也需要中继服务器
- PCP, UPnP (Port Control Protocol), (Universal Plug and Play) 协议, 请求 NAT 设备打开指定端口用于通信, 需要看受限网络中是否支持 (公司网, 校园网)
- WebRTC: 实时通信技术, 是一种基于 NAT 穿透实现的上层功能. 其实是使用 STUN 和 TURN, 不过不需要用户额外配置 (在应用中透明化).
- 
- 流行开源工具....

### WebRTC



WebRTC 建立连接时, 首先通过STUN (Session Traversal Utilities for NAT) 服务器直接连接客户端, 如果 NAT 和 防火墙 阻止了直接连接, 就会使用 TURN (Traversal Using Relays around NAT) 服务器 来穿透 NAT, 完成中继通信.

即内网穿透失败后, 两者不能之间建立点对点连接, 就会通过 TURN 中继服务器完成通信.

