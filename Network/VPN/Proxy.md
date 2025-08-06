---
copyright: JayWaves (2023)
revised: 24-07-12
---

## Proxy

目前流行的代理工具内核有: Clash, Xray, sing-box. 下面以 Clash 为例.

Clash 是一个代理工具, 负责将流量中继到代理节点, 需要搭配外部代理节点 (俗称机场, 有独立的服务提供商) 来绕过 GFW. 传统 VPN 协议则主动建立一个加密隧道来保护网络流量. 常见的 ClashN, ClashForWindows, ClashX 都是对命令行工具 clash 的UI 封装.

Clash 的主要协议是 Socks5, 它是一种代理协议, 负责中转和转发流量, **并不对数据加密**. 代理服务提供商则主要使用 Shadowsocks, 即加密的 Socks 协议, 来对流量加密. 还有其他可用的加密协议, 如: [Shadowsocks](Shadowsocks.md), VMess, Trojan, TLS/SSL. 

- 对于网路服务商 (ISP) 和中间人, 可以看到代理服务器的 IP. 但无法得知代理后的真实 IP 和内容.
- 对于代理服务器, 可以看到用户真实 IP. 若流量没有采用安全协议 (HTTPS), 也可以获得报文明文. **代理服务商可能由政府阻止假扮.**

```
virtual machines/apps on PC        
     \
      \ | socks5 | ip | tcp | https |
       \
      clash app on PC              
         \
          \ | shadowsocks | ip | tcp | https | (traffic encrypted)
           \
         remote proxy server    
              \
               \ | ip | tcp | https |  (payload encrypted only)
                \
            websites outside GFW
```


## 虚拟机配置代理

 1. 开启 vpn 客户端 (如 Clash) 的 LAN 模式

虚拟机 (linux) 和主机 (win10) 组成了一个局域网, 使用 `ipconfig` 在 cmd 中查看. LAN 模式让 clash 能够作为 proxy 转发局域网内的报文至真正的 proxy for vpn. 

2. 配置 proxy 环境变量, 可以写在 bashrc 中

clash 在局域网内仅接收 http 协议转发报文, 并且端口为 7890. 

clash 不接收 https 协议转发的报文, 因为本地内没有验证必要; 也别使用如 socks5 这种应用层之下的协议, 没有交互/错误处理/证书验证等能力, 容易出错, 至少我的虚拟机上会出现 TLS 验证问题.

host_ip 指的是主机 (win10) 的 ip. 我们将报文转发到该地址. **如果频繁更换 wifi (如一会 buaa wifi, 一会 buaa free wifi, 学校路由器会用 dhcp 从地址池给你分配一个动态地址), 主机的 ip 也是会不断跟着变化的. 更方便的办法是转发到主机上连接虚拟机的那个端口上, 以下是在 cmd 中执行 `ipconfig` 结果, 我将 wsl 虚拟机的以太网 ipv4 地址作为虚拟机 proxy 啦.**

```
无线局域网适配器 WLAN:
   连接特定的 DNS 后缀 . . . . . . . :
   IPv6 地址 . . . . . . . . . . . . : 2001:250:206:6cc3:98dc:5689:9402:aa67
   临时 IPv6 地址. . . . . . . . . . : 2001:250:206:6cc3:c13f:f0f:c0e7:1503
   本地链接 IPv6 地址. . . . . . . . : fe80::aa0:c9ce:8092:1a67%16
   IPv4 地址 . . . . . . . . . . . . : 10.195.208.xxx
   子网掩码  . . . . . . . . . . . . : 255.255.0.0
   默认网关. . . . . . . . . . . . . : fe80::b244:14ff:fe6a:c802%16
                                       10.195.0.1

以太网适配器 vEthernet (WSL):

   连接特定的 DNS 后缀 . . . . . . . :
   本地链接 IPv6 地址. . . . . . . . : fe80::b3ec:25ef:182f:cbd6%67
   IPv4 地址 . . . . . . . . . . . . : 172.18.240.1
   子网掩码  . . . . . . . . . . . . : 255.255.240.0
   默认网关. . . . . . . . . . . . . :
```

```bash
proxy="host_ip:host_port"
export $ALL_PROXY="http://$proxy"
```

不同软件针对不同协议检查的 proxy 环境变量不一样, 还可能是 `http_proxy` `https_proxy` `HTTP_PROXY` `HTTPS_PROXY`, 保险可以都设上. 我觉得 `ALL_PROXY` 够用 (区分大小写)

3. 一些软件用的是自己的 proxy, 不是系统的. 比如 go, git, 还有一些隐私浏览器和 IDE. 配置也类似步骤2

```shell
# git proxy
# git 可以使用 socks5, 因为 git 使用自己的 SSL
git config --global http.proxy "socks5://127.0.0.1:1080"
git config --global https.proxy "socks5://127.0.0.1:1080"

# 查看配置
git config --list


# go proxy
# go get 会先尝试使用自己的服务器, 通常被指定为坑爹的 goproxy.cn 实际上访问不了
# go get 使用 direct 时, 才会使用系统 proxy 访问 github 原仓库.
go env -w GOPROXY=https://goproxy.io,direct

# apt 有时不使用系统代理 http_proxy, 单独设置 /etc/apt/apt.conf
Acquire::http::proxy "http://127.0.0.1:8000/";
Acquire::ftp::proxy "ftp://127.0.0.1:8000/";
Acquire::https::proxy "https://127.0.0.1:8000/";

```

> 参考 [机场+VPN](https://playbeasts.com/question/2691)

## 其他配置

TUN 模式: 工作于[网络层](../NetworkL3/IP.md). 处理 IP 层及以上的协议.

TAP 模式: 工作于[数据链路层](../数据链路层.md), 处理以太网帧, 模拟真实的网卡环境, 如虚拟机组网和桥接网络.

### TUN 模式

创建一个虚拟网卡 (utun0), 在应用层就拦截和重定向系统的网络流量. 

有几个重要特点:
1. 所有流量 (TCP, UDP, ICMP, DNS) 都会被截获, 因此都会被代理软件的规则 (Rules) 处理
2. 某些应用 (系统更新, 后台服务) 会忽略系统代理, TUN 可以强制接管. 因此其对流量的控制也精准. 
3. **hosts 文件和防火墙规则可能被 TUN 模式直接覆盖, 应为流量在应用层就被接管.**

## Allow Lan 

allow lan 模式除了允许本地 `127.0.0.1` 回环网段设备访问 Clash 代理, 也可以允许本地局域网网段 (如 `192.168.31.0`) 访问. 但是注意 PC 上配置好防火墙安全规则, 仅允许**专用网络**的特定地址入站, 并仅访问特定端口 (如 `7890`).

## shadowsocks

下面介绍使用 `shadowsocks` 命令行工具, 实现科学上网. 不是因为它"方便", 只是因为它"可以".

从代理服务器提供商处, 获取订阅地址:
```url
https://rssteacat1.xyz/api/v1/client/subscribe?token=88c6e9947da241486d4e7662b6e60adc
```

在浏览器访问该订阅地址, 获得一大串 base64 代码:

```base64
c3M6Ly9ZV1Z6TFRJMU5pMW5ZMjA2Wm1VeE5Ua3hOMll0TURFME15MDBZVEptTFdJME1UVXRabVUzTjJNeE16Z3dPREkzQGhrMS50ZWFjYXQ0Lnh5ejo1MDA2NyMlRTklQTYlOTklRTYlQjglQUYxDQpzcz
...
KbUxXSTBNVFV0Wm1VM04yTXhNemd3T0RJM0BneS50ZWFjYXQ0Lnh5ejo1MDA4MSMlRTUlQjclQjQlRTglQTUlQkYNCg==
```

解码后, 获得 shadowsocks 协议地址. 
```url
ss://YWVzLTI1Ni....zgwODI3@hk1.teacat4.xyz:50067#%E9%A6%99%E6%B8%AF1

...
```

此时 `//...@` 之间仍是一串 base64 编码, `#` 后是一串 URL 编码, 解码后获得:
```url
ss://aes-256-gcm:fe15917f-0143-4a2f-xxx-380827@hk1.teacat4.xyz:50067#香港1
```

每部分解释如下:
- `ss://` shadowsocks 协议头
- `aes-256-gcm` 指加密方法
- `fe169...827` 指登录代理服务器的密码
- `hk2.teacat3.xyz` 指代理服务器的地址
- `50067` 指代理服务器的端口
- `#香港2` 标签名称, 指该代理服务器位于香港.

根据上述协议内容写配置文件. 或者直接输入 `ssurl.exe` 工具解析为 JSON.

```json
{
	"server": "...",
	"server_port": 50067,
	"local_address": "127.0.0.1",
	"local_port": 1080,
	"password": "...",
	"timeout": 300,
	"method": "aes-256-gcm",
	"fast_open": false // TCP Fast Open 配置, 需要双方皆支持
}
// 注意, 标准 JSON 文件不支持注释~~
```

之后启动客户端 `ss-local` 即可开启代理. 可以将浏览器或者 Shell 的代理配置为 `127.0.0.1:1080`, 从而使用 shadowsocks 代理. *当然了, 浏览器肯定也支持直接配置使用 Socks 访问远程的代理, 不需要再经过本地的 shadowsocks 跳板*.

```sh
ss-local -c ~/shadowsocks.json
```

