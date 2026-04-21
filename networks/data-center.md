![|500](http://oss.jay-waves.cn/til/legacy-data-center-network.avif)

## Legacy DCN 

经典树状 DCN ，分为三层：
1. Access Layer. A group of hosts(mostly 16/32) is connected to one switch, called ToR switch (Top of Rack)
2. Aggregation Layer. uses high-end switches aggregating the traffic
3. Core Layer 

Core --> Aggregation --> Access 

Over-subscription: total allocated or promised resources exceed the actual available capacity. providers oversubscribe bandwidt because it's unlikely that all users will consume their maximum bandwidth simultaneously.

数据中心优化的原则：
* 减少过订阅，在极端情况下能尽量满速。
* scale-out better than scale-up. 升级昂贵的高速交换机，不如规模化低成本交换机，网络规模线性增长。
* 提供等价路径 (Equal-Cost Paths, ECMP) 减少单路径热点瓶颈
* 容错和路由错误排查

## Clos-based (Fat Tree)

#### Goolge Fat Tree 

![|500](http://oss.jay-waves.cn/til/google-fat-tree-topo.avif)

Fat Tree 提供更多的并行链路，让上层的总带宽变胖，从而避免 Over-Subscription.

#### Facebook Fat Tree 



## BCube 

![|400](http://oss.jay-waves.cn/til/bcube-data-center-topo.avif)

BCube 是递归结构，设每个交换机端口数为 $n$ ，每台服务器需要 $k$ 个 NIC 网卡，网卡 $i$ 对应 $BCube_{i}$ 层内的 $i$ 交换机（同层跨集群计数）。$BCube_{0}$ 有一个交换机，星型拓扑连接 $n$ 个服务器，编号分别为 $1\dots n$ 

$BCube_{k}$ 层有 $k$ 个交换机，有 n 个 $BCube_{k-1}$ 子集群。$BCube_{k}$ 层第 $k$ 个交换机的第 $i$ 个网口，连接第 $i$ 个 $BCube_{k-1}$ 集群的第 $k$ 个主机。

$k$ 个网卡的服务器，配 $n$ 个端口交换机，最多在网络中容纳 $n^{k}$ 个主机。

BCube 设计中，由服务器决定转发路径。一个服务器有 K+1 个网卡（或 L2 交换机），服务器和路径中间服务器共同决定转发路径。因此，BCube 是 *Server-Centric* 架构。

## Jellyfish 

## F10

## 参考

综述 @lebiednik2016

文章: 
- @Singh2015. jupiter rising, a decade of Clos Topologies and Centralized Control in google's DAtaCenter Network. Goolge.
- @Greenberg2009. VL2, a scalable and flexible Data Center Network. Microsoft.