Fedora 更新比 Debian 系更快, 是 RHEL 的上游版本. 

## 包管理 DNF/RPM

## 访问控制

Fedora 默认开启 [Selinux](../../可观测性/访问控制.md)

## 网络服务

系统服务管理: `systemd`

防火墙: `firewalld` 

### 配置代理

修改 `/etc/systemd/system.conf`

```conf
DefaultEnvironment="http_proxy=http://127.0.0.1:7890" \
                   "https_proxy=http://127.0.0.1:7890" \
                   "ftp_proxy=http://127.0.0.1:7890" \
                   "no_proxy=localhost,127.0.0.1,::1"
```

刷新 systemd 服务 `sudo systemctl daemon-reexec`

修改 dnf 的代理: `/etc/dnf/dnf.conf`

```conf
proxy=http://127.0.0.1:7890
```