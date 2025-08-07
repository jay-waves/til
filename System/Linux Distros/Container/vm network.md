## Docker

docker 默认网桥模式: 虚拟一个网桥 (docker0), 为每一个容器分配一个IP地址 (container ip), 网桥是每个容器默认网关

| 网络模式  | 配置                           | 说明                                                                                                  |
| --------- | ------------------------------ | ----------------------------------------------------------------------------------------------------- |
| host      | --network host                 | 容器和宿主机共享 Network namespace. **注意,共享网络不等于共享代理**                                                                    |
| container | --network container:NAME_OR_ID | 容器和另外一个容器共享 Network namespace                                                              |
| none      | --network none                 | 容器有独立的 Network namespace，但并没有对其进行任何网络设置，如分配 veth pair 和网桥连接，配置 IP 等 |
| bridge    | --network                      | bridge 默认模式, 网络隔离, 但分配一个内部IP. 构建时也使用该网络模式.                                                                                       |


> 部分软件不走 系统/Shell 代理, 需要单独设置代理. 比如 Go, Git, apt. 代理可以配置为宿主机本地网络上的代理软件. 如 [Proxy](../../../Network/VPN/Proxy.md)

隔离网络:
```shell
# 用户自定义网络时, 默认和宿主网络是隔离的
docker network create <my_network>

docker run ... --network <my_network>
```

## VM

| 模式                    | 虚拟机</br> <-> </br>宿主机 | 虚拟机</br> <-> </br>虚拟机 | 虚拟机</br> <-> </br>外部网络 | 说明                                             |
| ----------------------- | --------------------------- | --------------------------- | ----------------------------- | ------------------------------------------------ |
| 桥接模式  (Bridged)     | 是                          | 是                          | 是                            | 虚拟机扮作局域网的新物理主机, 可以获取局域网分配的实际 IP, 可被局域网发现.             |
| NAT                     | 是                          | 是                          | 是 (通过宿主)                 | 虚拟机通过宿主机连接访问外部网络, 虚拟机有虚拟IP |
| 内部网络模式 (Internal) | 否                          | 是                          | 否                            | 仅虚拟机之间通过专用虚拟网络通信                                                |
| 仅主机 (Host-Only)      | 是                          | 是                          | 否                            | 虚拟机和主机间有虚拟网络接口, 但无法访问外部网络                                                |

桥接模式下, 本地物理网卡和虚拟网卡通过 VMnet0 虚拟交换机进行桥接, 在拓扑图中处于同等地位, 因此两者的 IP 地址也在同一网段.

在 NAT 模式下, 虚拟机程序模拟一个 NAT 服务器 (VMware VMnet8). 虚拟机的网卡连接到 NAT 服务器, 再通过 NAT 连接宿主机网络. 此时, 如果要访问虚拟机内 80 端口服务, 需要在 NAT 服务器上配置端口映射, 将宿主机的某个端口映射到虚拟机的 80 端口.

在仅主机模式下, 虚拟网络是完全封闭的网络, 宿主机也有一个虚拟网卡在该网络内. 但是宿主机不再提供 NAT 服务, 因此虚拟网络无法联通外部互联网.

### 例子

宿主机是: 

```
1. lo: <LOOPBACK> state UNKNOWN
2. enp44s0: <NO-CARRIER,BROADCAST,MULTICAST> state DOWN link/ether ....
3. enp44s0: <NO-CARRIER,BROADCAST,MULTICAST> state DOWN link/ether ....
4. wlo1: <BROADCAST,MULTICAST> state UP link/ether ... inet 192.168.31.232/24 brd 192.168.31.255 dynamic
5. virbr0: state DOWN ... 
```

在虚拟机中, 模拟的网络设备通常有几种:
- TUN 设备: 虚拟三层网卡, 工作在网络层 , 用于处理 IP 数据包
- TAP 设备: 虚拟二层网卡, 工作在数据链路层, 用于处理以太网帧 (L2)
- Bridge 设备: 虚拟交换机

对于 ethernet 网卡 (物理网线), 可以连接到一个桥接器 (Bridge) 上, 桥接器上
再连接多个 TAP 设备, 从而在一台主机上虚拟出多个物理网卡 (有实际 IP 地址).
