## Network Info

windows 上原生网络工具请见 [cmd/网络](../Cmd/网络.md)

### `ip`

`ip addr show`

### `ping`

```bash
ping host
```

### `mtr`

[`mtr`](http://www.bitwizard.nl/mtr/), my traceroute, = traceroute+ping. mtr 用于跟踪路由, 提供了交互式界面, 可以灵活调整参数.

### `netcat`, `socat`

`netcat` (or `nc`) 用于读取和写入网络连接. 常见用途为端口扫描, 监听传入连接, 端口重定向和测试. 当然, 端口扫描更专业的工具是 `nmap`, `netcat` 监听链接时有安全问题.

```sh
nc -l 1234           # 监听端口, 作为服务器
nc example.com 1234  # 连接到主机端口, 作为客户端
```

`socat` 是加强版 `netcat`:

```sh
# 转发 tcp 端口
socat TCP-LISTEN:port1,fork TCP:target.address:port2
```

### `nslookup` 

根据域名查看 ip.

### `dig` & `host`

查询 domain 的 DNS 信息.

```bash
dig domain
```

### `whois`

Gets whois information for domain.  

```bash
whois domain
```

## Network Debug

### `netstat`, `ss`

`netstat` (network statistics) 用来显示套接字信息.

- `t` 显示 TCP 连接
- `u` 显示 UDP 连接
- `l` 仅显示处于监听状态的服务端口
- `a` 显示所有套接字 (包括非监听状态的)
- `n` 显示地址和端口号的数字形式 (不解析为名称)
- `p` 显示每个连接对应的进程ID和进程名

```bash
# 监听端口进程
netstat -lntp
ss -plat

# 用 lsof 也可实现, 显然更麻烦.
lsof -iTCP -sTCP:LISTEN -P -n
```

`ss` (socket statistics) 比 `netstat` 更加现代, 处理速度更快.

### `iftop`, `nethogs`,`slurm`

[`iftop`](http://www.ex-parrot.com/~pdw/iftop/) 或 [`nethogs`](https://github.com/raboof/nethogs) 用于查找正在使用带宽的套接字连接或进程. 其中 nethogs 可以将流量分解到进程, 诊断带宽使用时更有效. slurm 则用于对总体流量变化进行可视化, 更简洁.

iftop 侧重于持续监控网络连接, netstat 则用于一次性检查网络配置和状态.

### `wireshark`, `tshark`

[`wireshark`](https://wireshark.org/), [`tshark`](https://www.wireshark.org/docs/wsug_html_chunked/AppToolstshark.html) 和 [`ngrep`](http://ngrep.sourceforge.net/) 都是用于截获过滤报文的工具. 其中 tshark 和 ngrep 是命令行工具.

该系列工具用于分析**报文具体协议和内容**, 而 `iftop` 则仅用于监控流量.

## Network Connection Tool

### `scp`

和远程主机间传输文件.

```bash
# 从本地拷贝到远程
scp source_file user@host:directory/target_file

# 从远程拷贝到本地:
scp user@host:directory/source_file target_file
scp -r user@host:directory/source_folder target_folder

# 链接指定端口
scp -P port user@host:directory/source_file target_file
```

### `rsync`

和 `scp` 一样用于跨设备传输文件, 但可以**增量传输**, 常用于大量文件的同步. `rsync` 还可以实现文件续传, 而不用从头开始.

```bash
rsync source_folder user@host:target_folder
rsync user@host:target_folder target_folder
```

本地 `rsync` 有个神奇用法, 就是[删除大量文件时速度很快](https://web.archive.org/web/20130929001850/http://linuxnote.net/jianingy/en/linux/a-fast-way-to-remove-huge-number-of-files.html)

```sh
mkdir empty && rsync -r --delete empty/ some-dir && rmdir some-dir
```

### `ssh`

请查看: [Network/应用层/ssh](../../Network/应用层/SSH.md)

### `curl`

或者试试更潮的 [httpie](https://github.com/jkbrzt/httpie)

```sh
# 网络工具不自动使用系统代理
curl https://www.google.com -x http://127.0.0.1:7890
curl -v \
	--resolve github.com:443:140.82.112.4 https://github.com \
	-x http://127.0.0.1:7890
```

常用参数:
- `o`: 输出到其他文件 `curl -o myfile.txt`
- `H`: 添加 http 头 `curl -H "User-Agent: MyUserAgent" URL`
- `I`: 仅获取 http 头部 `curl -I URL`
- `u`: 用户认证 `curl -u username:password URL`
- `--limit-rate`: 限制速度
- `x`: 使用代理 `curl -x http://proxy-server:port http://www.example.com`
- `k`: 跳过 SSL 验证.
- `X`: 使用其他请求方法, 如 Post. `curl -X POST -d "user=yjw&passwd=123456" URL`
- `L`: 让 curl 跟随服务器重定向.
- `--resolve example.com:443:1.2.3.4` 手动指定域名的dns解析

### `wget`

`curl` 的亲兄弟, 不过一般只用来下载文件.

<pre>
$ wget file_url
</pre>

### `unshare`

unshare 命令运行运行子程序时与父进程的资源分离, 比如分离网络命名空间.

```bash
sudo unshare --net /path/to/program
```

### `firejail`

...