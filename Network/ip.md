## IPv4



## IPv6

ipv6 和 ipv4 是不兼容的, 这导致了从 ipv4 向 ipv6 过渡困难.

## host & DNS

windows 上 hosts 文件: `C:\Windows\System32\drivers\etc\hosts`

linux: `/etc/hosts`

### 常见域名

```
localhost                  127.0.0.1 (IPv4)
localhost                  ::1 (IPv6)
```

#### IPv4 转发

临时开启: `sudo sysctl -w net.ipv4.ip_forward=1`

永久开启: 

- 修改 `/etc/sysctl.conf`, 添加 `net.ipv4.ip_forward = 1`
- 重启服务 `sudo sysctl -p`

