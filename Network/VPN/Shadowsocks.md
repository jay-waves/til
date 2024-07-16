---
os: Win10
author: JayWaves
date: 24-07-12
---

## shadowsocks

本篇旨在介绍如何使用 `shadowsocks` 和 `clash` 的命令行工具, 实现科学上网. 不是因为它"方便", 只是因为它"可以".

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

## clash

clash 也是命令行的代理工具. 配置使用 `.yaml` 文件, 而且有 API 供外部访问和调整代理配置信息. 所以包装 clash 的 UI 软件更多.