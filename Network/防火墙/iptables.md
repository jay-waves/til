## IPTables

linux 内核防火墙配置工具, 应用于 Kernel Netfilter 框架中.

配置结构: tables -> chains -> rules

iptables 内置五张表 (tables):
- filter: 过滤数据包, 包括内建链: `INPUT`, `OUTPUT`, `FORWARD`
- nat: 网络地址转换, 包括内建链: `PREROUTING`, `POSTROUTING`, `OUTPUT`
- mangle: 处理特定数据包, 包括链: `PREROUTING`, `POSTROUTING`, `INPUT`, `OUTPUT`, `FORWARD`
- raw: 处理异常, 包括链: `PREROUTING`, `OUTPUT`
- security: 配置强制访问控制网络规则, 包括 `INPUT`, `OUTPUT`, `FORWARD`

iptables 有五条链 (chains), 用来表示数据流向, 用户可以向链中添加规则. 
- PREROUTING 路由前链, 数据包刚达本机, 处理路由规则前, 常用于目的地址转换.
- INPUT 输入链, 发往本机的数据包通过
- OUTPUT 输出链, 从本机发出的数据包通过
- FORWARD 转发链, 本机转发的数据包通过
- POSTROUTING 路由后链, 数据包利卡本机时通过, 常用于源地址转换(SNAT)

iptables 规则链流向如下:

```
		 Network
-------------------------
|                       +
+                       |
PREROUTING              POSTROUTING
|                       +
+                       |
Routing -- + FORWARD -- +
|                       |
+                       |
INPUT                   OUTPUT
|                       +
+                       |
-------------------------
	  Local Process

                                                     -- +  指箭头
```

## ufw

- `sudo ufw enable`
- `sudo ufw disable`

规则:
- `sudo ufw allow ssh`
- `sudo ufw allow 22/tcp` 允许端口 22 的 tcp 报文.
- `sudo ufw allow from 192.168.1.10 to any port 22`
- `sudo ufw deny 1234/tcp`
- `sudo ufw delete [rule_number]`, rule_number 通过 `status` 命令查看.**