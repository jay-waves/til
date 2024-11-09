开放最短路径优先协议 (Open Shortest Path First, OSPF) 是一个**内部网络协议**, 用于在单一自治系统 (AS) 内实施路由决策.

每个 OSPF 路由器都保存有全网链路信息的链路状态 (Link-State) 数据库, 通过 [Dijkstra 算法](../../Algorithm/图/最短路径算法.md)计算路由表.

OSPF 有比较严格的认证机制.