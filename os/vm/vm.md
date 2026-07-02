## 云服务

| 层次         | 云服务 |
| ------------ | ------ |
| 软件         | SaaS   |
| 开发平台     | PaaS   |
| 抽象计算资源 | IaaS   |
| 虚拟化层     |        |
| 物理硬件     |        |
| 网络基础设施             |        |



## 虚拟机类型

![](../../attach/vm-type.avif)

type-1 hypervisor (bare-metal hypervisor) 直接管理宿主机硬件, 本身是最底层 OS. 如 KVM, Xen, ESXi.

type-2 hypervisor 则是 Host OS 和 Guest OS 间的中间层. 所有 I/O 都要走 Host OS. 如 VMware Workstation, VirtualBox.

container. 不虚拟硬件, 也不提供内核, 只在内核基础上进行资源隔离. 

### 特权指令处理

## [Docker](docker.md) 

* Linux [Namespaces & Control Groups](linux-ns&cgroup.md) 
* Union File SYstem (OverlayFS): Read-only Image Layers + a Writable Container Layer 

## 参考

A study of security isolation techniques. @shu2016.