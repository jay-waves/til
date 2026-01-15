## 云服务

![|500](../../../attach/Pasted%20image%2020240924224540.avif)

## 虚拟机类型

![](../../attach/vm-type.avif)

type-1 hypervisor (bare-metal hypervisor) 直接管理宿主机硬件, 本身是最底层 OS. 如 KVM, Xen, ESXi.

type-2 hypervisor 则是 Host OS 和 Guest OS 间的中间层. 所有 I/O 都要走 Host OS. 如 VMware Workstation, VirtualBox.

container. 不虚拟硬件, 也不提供内核, 只在内核基础上进行资源隔离. 

### 特权指令处理



## 参考

A study of security isolation techniques. @shu2016.