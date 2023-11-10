## 网络

docker 默认网桥模式: 虚拟一个网桥(docker0), 为每一个容器分配一个IP地址 (container ip), 网桥是每个容器默认网关

| 网络模式  | 配置                           | 说明                                                                                                  |
| --------- | ------------------------------ | ----------------------------------------------------------------------------------------------------- |
| host      | --network host                 | 容器和宿主机共享 Network namespace                                                                    |
| container | --network container:NAME_OR_ID | 容器和另外一个容器共享 Network namespace                                                              |
| none      | --network none                 | 容器有独立的 Network namespace，但并没有对其进行任何网络设置，如分配 veth pair 和网桥连接，配置 IP 等 |
| bridge    | --network                      | bridge 默认模式                                                                                       |

直接将宿主机设置为proxy即可, 如用[clash](../Network/proxy.md), 设置 `all_proxy=http://host_ip:7890`

## 基础

ros2 docker:

```
docker pull osrf/ros:foxy-desktop

docker run -it osrf/ros:foxy-desktop
```

### 工具

docker 中一些常见工具不能正常安装:

ping: `apt install iputils-ping`

`apt install net-tools`