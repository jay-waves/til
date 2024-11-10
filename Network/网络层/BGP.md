边界网络协议 (Border Gateway Protocol, BGP) 是一种外部网关协议 (EGP), 用于互联网自治系统 (AS, Autonomous System) 之间的路由交换. 属于互联网的 "骨干协议", 用于全球范围的路由信息传递. 

BGP 使用路径矢量算法, 每条路由信息包含经过的所有自治系统列表. 允许管理员基于策略控制路由选择, 如考虑政府政策因素的路由控制.

BGP 缺乏安全的路由认证机制.

**BGP (Border Gateway Protocol) , OSPF (Open Shortest Path First) 和 RIP (Routing Information Protocol) ** 是三种常见的路由协议, 用于在不同规模的网络中管理路由信息. 这三者各自有不同的应用场景和特性: 

| 区别     | [BGP](Network/网络层/BGP.md) | [RIP](Network/网络层/RIP.md) | [OSPF](Network/网络层/OSPF.md) |
| -------- | ---------------------------- | ---------------------------- | ------------------------------ |
| 算法     | 路径矢量算法                 | 链路状态算法                 | 距离矢量算法                   |
| 应用范围 | EGP, 互联网级别大型网络      | IGP, 小型自治系统内部        | IGP, 中型自治系统内部          |
| 收敛速度 |                              | 较慢                         | 较快                               |
