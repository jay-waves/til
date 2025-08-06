### 配置本地 Hosts

windows 上 hosts 文件: `C:\Windows\System32\drivers\etc\hosts`

linux: `/etc/hosts`

### 使用企业 DNS 服务

默认使用 ISP (网络服务提供商, 如电信或移动) 的 DNS 服务器. 可能有安全或速度问题, 比如政府蓄意对 DNS 解析进行污染投毒. 企业和云服务商也提供公共 DNS 服务, 如谷歌和阿里云.

```sh
# 使用 Goolgle DNS 查询域名
nslookup example.com 8.8.8.8

# 查询默认 DNS 服务商, Linux 上还可以查询 /etc/resolv.conf
nslookup
```

### 反向 DNS 查询

从 IP 地址反向查询对应域名.

```sh
# windows
nslookup 8.8.8.8

# linux (apt install dnsutils)
dig -x 8.8.8.8

# linux 
host 8.8.8.8
```

一些网页服务, 如 ipinfo.net, 查询结果更精准.

***

## DNS

The Domain Name System (DNS) translates human-readable domain names, like `www.example.com`, into machine-readable [IP](../NetworkL3/IP.md) address, such as `192.0.2.1`. 

DNS 使用 [UDP](../TransportL4/UDP.md) 协议 53 端口.

### Key Components

Domain Names: In `blog.csdn.cn`, `.cn` is the top-level domain (TLD), `csdn` is the second level domain, `blog` is a subdomain.

**DNS Records**: 指存储在 DNS 服务器中的记录信息, 有很多种类: 
- **A**: 将一个域名映射为 IPv4 地址
- **AAAA**: 将一个域名映射为 IPv6 地址
- **CNAME**: 将一个域名映射到另一个域名, 类似域名的别名. 比如 `www` 映射到根域名.
- **MX**: 指定某个权限域名服务器下的邮件服务器的地址.
- **TXT**: 存储文本信息, 一般用于安全性配置. 
- **NS**: 指定负责解析的 DNS 服务器 (DNS 委派).
- **PTR** (Pointer Record): 反向 DNS 解析. 当来源 IP 的 PTR 和对应的 A 记录不一致时, 可能被归类为恶意活动 (如垃圾邮件).

```dns
example.com.    3600    IN    A    93.184.216.34
example.com.    3600    IN    AAAA    2606:2800:220:1:248:1893:25c8:1946
www.example.com.    3600    IN    CNAME    example.com.
example.com.    3600    IN    MX    10 mail.example.com.
example.com.    3600    IN    MX    20 backup.example.com.
example.com.    3600    IN    TXT    "v=spf1 include:_spf.google.com ~all"
example.com.    3600    IN    NS    ns1.example.com.
example.com.    3600    IN    NS    ns2.example.com.
example.com.    3600    IN    SOA    ns1.example.com. admin.example.com. 2023101501 7200 3600 1209600 3600
34.216.184.93.in-addr.arpa.    3600    IN    PTR    example.com.
```

![](../../attach/Pasted%20image%2020240802105050.avif)

| 根域名服务器   | Root DNS Server          |
| -------------- | ------------------------ |
| 权威域名服务器 | Authoritative DNS Server |
| 本地域名服务器 | Recursive DNS Resolvers                         |

### DNS Resolution

递归查询指请求方将整段域名 `gist.github.com` 的解析交付给另一个域名服务器完成, 自身仅等待结果, 不再负责后续转发; 迭代查询指请求方自己完成查询消息的转发, 先请求根域名服务器, 根域名服务器返回 `.com` 服务器的地址, 请求方再向 `.com` 域名服务器查询 `github.com` 的地址, 以此类推. 迭代查询的优点是便于本地服务器缓存信息. 实践中, 客户端向本地服务器请求使用的是递归查询 (使用 UDP 协议), 而本地服务器则使用迭代查询 (使用 TCP 协议).

![](../../attach/dns%20query.avif)

在企业内网中, 会有独立的 DNS 服务器充当本地域名服务器. 如果企业有对外服务, 就会有对外的权威域名服务器, 用于提供自身域名下的子域名解析. 如 Github 等公司, 会使用 Cloudflare 这类第三方服务, 提供 DNS, CDN, 负载均衡和安全防护.

在家庭网络中, 路由器充当网关组成小型的 LAN, 所以 DNS 会先转发给路由器, 路由器再转发给网络供应商 (如电信, 移动) 的域名服务器, 由其充当本地域名服务器.

![](../../attach/dns%20local%20query.avif)
