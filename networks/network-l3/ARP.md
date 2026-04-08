## ARP协议

ARP: address resolution protocol, 地址解析协议.   
用于从 [IPv4](IPv4.md) 地址寻得其映射的硬件 [MAC](../data-link-l2.md) 地址, 寻找方式是向网络中发送 ARP 请求.

每台主机缓存 ARP 表, 记录 `<IP, MAC, TTL>` 映射关系, TTL为有效期.

