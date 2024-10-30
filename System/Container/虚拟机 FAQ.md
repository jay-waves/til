#FAQ 

## 端口占用导致无法启动 Docker 容器

https://stackoverflow.com/questions/65272764/ports-are-not-available-listen-tcp-0-0-0-0-50070-bind-an-attempt-was-made-to

管理员模式运行CMD, 重启一下Winnat, 把端口腾出来. 可能是Clash把本地端口占满了没释放.

```
net stop winnat
net start winnat
```

## 固定体积的虚拟机vdi扩容:

提前声明占有所有磁盘体积的vdi, 理论上并不能扩容. 要实现这一点, 必须新建一个 **动态增长磁盘体积(而不提前占有)** 的新vdi, 并将旧硬盘克隆过去, 步骤如下:
1. 去 VB 软件 Virtual Media Manager 中新建一个动态增长内存的 vdi
2. 将虚拟机原vdi从挂载中卸载
3. `VBoxManage clonemedium \path\to\old\vdi \path\to\new\vdi --existing`, VBoxManage 为 VB 安装目录的命令行工具.
4. 挂载新 vdi