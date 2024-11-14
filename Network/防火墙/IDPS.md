Intrusion Detection and Prevention Systems (IDPS). IDS 是被动的, 监控网络流量和系统行为, 并将威胁和可疑行为告知管理员. IPS 是主动的, 检测可疑流量, 并主动拦截. IDS 一般在旁路部署, 监控流量镜像; IPS 直接部署在流量路径中.

```
入侵响应 ---> 远程管理
   ^           |
   |           |
   |           |
入侵分析 <----- +
   ^
   |
   |
事件提取
```

按数据来源分类:
- NIDS: 基于网络的入侵检测系统. 截获网络数据包, 提取特征并与知识库中攻击相比较.
- HIDS: 基于主机的入侵检测系统. 通过分析监控*日志和审计记录*来发现误操作.
- DIDS: 分布式入侵检测系统. 一般指融合前两者功能.

按检测技术分类:
- 误用检测: 假设所有入侵行为都能表达为某种特征或模式, 收集实际行为的信息与知识库 (异常行为描述库) 相比较. 也称为*基于知识的异常检测*.
- 异常检测: 测量正常行系统为的基线, 并将实际行为与基线 (正常行为描述库) 相比较. 也称为*基于行为的异常检测*, 也称为*基线检查*.
- 完整性检测

IDS 评价指标:
- 检测入侵能力, 包括知识库的完善程度.
- 抗欺骗能力, 指误报率和漏报率.
- 远程管理能力
- 自身安全性, 性能不能称为网络瓶颈, 并且满足各类安全需求.

## IDS

RFC 4949, P166

> intrusion detection system (IDS)
> 1. (N) A process or subsystem, implemented in software or
> hardware, that automates the tasks of (a) monitoring events that
> occur in a computer network and (b) analyzing them for signs of
> security problems. [^1]
> 
>  2. (N) A security alarm system to detect unauthorized entry.
> [DC6/9].

> Tutorial: Active intrusion detection processes can be either host-
> based or network-based:
> -  "Host-based": Intrusion detection components -- traffic sensors
>    and analyzers -- run directly on the hosts that they are
>    intended to protect.
> -  "Network-based": Sensors are placed on subnetwork components,
>    and analysis components run either on subnetwork components or
>    hosts.

[^1]: [NIST SP800-31 (IDS)] was superseded by [NIST SP 800-94 (IDPS)] on 2007