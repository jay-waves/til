SSL (Secure Sockets Layer, 安全套接层), 及其继任者 TLS (Transport Lyayer Security, 传输安全) 提供了 TCP/IP协议 与 应用层协议 间的透明安全协议. 现在 ssl 多代指 tls.

tls 握手过程:
1. client hello: tls 版本, ClientRandom
2. server hello: 协议版本, 密码套件, ServerRandom
3. server certificate.
4. client, server 交换必要参数, 生成预主密钥 (Pre-Master Secret).
5. (optional) client certificate
6. 生成会话密钥 (Master Secret), 用 ClientRandom, ServerRandom, Pre-Master Secret

### vpn

虚拟专用网络, Virtual Private Network, 使用加密隧道协议建立专用网络连接.

常用隧道技术:
- PPTP, L2TP: 数据链路层协议
- IPSec: 网络层协议, 是网关-网关vpn
- SSL/TLS: 应用层协议, 是远程访问vpn