## ARP协议

ARP: address resolution protocol, 地址解析协议.   
用于从 [IP](IP.md) 地址寻得其映射的硬件 [MAC](../数据链路层.md) 地址, 寻找方式是向网络中发送 ARP 请求.

每台主机缓存 ARP 表, 记录 `<IP, MAC, TTL>` 映射关系, TTL为有效期.

