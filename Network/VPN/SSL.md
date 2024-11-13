SSL (Secure Sockets Layer, 安全套接层)[^1], 及其继任者 [TLS (Transport Lyayer Security, 传输安全)](https://datatracker.ietf.org/doc/html/rfc4346) 提供了 TCP/IP 协议与应用层协议之间的透明安全协议, 实现服务器和客户端之间的机密通信和对等认证 . 现在 SSL 多代指 TLS.

[^1] 最初由 Netscape Communications Inc. 开发, 后发布多个改进版本.

| 网络层级 | 协议             |
| -------- | ---------------- |
| 应用层   | smtp, http, nntp |
| 安全层, 应用层, 表示层   | TLS              |
| 传输层   | TCP          |
| 网络层   | IP                 |

TLS/SSL协议步骤:
- 认证
- 密钥协商
- 数据加密: 保密性和完整性

TLS/SSL协议分为上下两层:
1. (上层) 握手层协议 (handshake layer protocal, SSL Change Cipher Spec Protocol, SSL Alert Protocol)
2. (下层) 记录层协议 (record layer protocal)

### 记录层协议

功能: (按顺序) 
1. 分段
2. 压缩 (存在安全缺陷, TLSv1.3 已不支持)
3. 增加 MAC 
4. 加密 
5. 增加 SSL 记录

### 握手层协议

功能:
- 对等实体验证: 为用户验证服务器身份, (可选) 为服务器验证用户身份.
- 协商 TLS 版本和加密套件
- 使用密钥协商算法生成:
	- 一个共享的会话密钥, 用于数据保密性服务
	- 一个加密哈希, 用于数据完整性服务

密码学套件 (Cipher Suite), 如 `TLS_DHE_RSA_WITH_AES_256_CBC_SHA256`
- DHE: 密钥交换算法
- RSA: 身份认证算法
- AES_256_CBC: 对称加密算法
- SHA256: 消息摘要算法

```
+--------+                                      +--------+
|        o ---- ClientHello ------------------> |        |
|        |                                      |        |
|        |       ServerHello                    |        |
|        |       [Certificate]                  |        |
|        |       [ServerKeyExchange]            |        |
| Client |       [CertificateRequest]           | Server |
|        |       ServerHelloDone                |        |
|        | <----------------------------------- o        |
|        |                                      |        |
|        |       [Certificate]                  |        |
|        |       ClientKeyExchange              |        |
|        |       [CertificateVerify]            |        |
|        |       ** ChangeCipherSpec **         |        |
|        |       Finished                       |        |
|        o -----------------------------------> |        |
|        |                                      |        |
|        |       ** ChangeCipherSpec **         |        |
|        |       Finished                       |        |
|        < ------------------------------------ o        |
|        |                                      |        |
+--------+                                      +--------+
```

```
     Client                               Server
     ^ ClientHello
     | + key_share*
Key  | + signature_algorithms*
Exch | + psk_key_exchange_modes*
     v + pre_shared_key*   ----->
                                     ServerHello           ^ Key Exch
                                     + key_share*          |
                                     + pre_shared_key*     v
                                     {Ecrypted Extensions} ^  Server
                                     {CertificateRequest*} V  Params
                                     {Certifacte*}         ^
                                     {CertifacateVerify*}  | Auth 
                                     {Finished}            v
                          <------    {App Data*}
     ^ {Certifacate*}
Auth | {CertificateVerify*}
     v {Finished}
       {App Data}         <----->  {App Data}
```

