## 设备发现

内核启动时, 先加载*总线驱动*. 总线驱动将扫描总线上的所有设备, 如果得到设备响应,
就会注册一个对象 `struct device`, 并挂载到对应总线下面.
- pci_dev
- usb_device
- i2c_client ...

设备的种类繁多, 硬件能力也多样, 使用 DMA, IRQ, MMIO, ACPI 等等方式和主机通信. 

## 驱动注册

内核发现*设备*后, 会注册与设备型号匹配的具体*驱动*, 然后向上层提供接口:
- 块驱动 (nvme.ko, sd.ko) 会为 PCIe, NVMe, SSD 设备注册 gendisk
- 字符驱动会为串口注册一个 tty 接口
- 网络驱动也会注册 net_device 

注册驱动时, 调用 `device_create(my_class, parent, dev_num, ..., "my_dev")`, 就会在
`/sys/class/my_class/my_dev/` 下生成 sysfs 节点. 用户空间守护进程 udev 监听到 `uevent`, 会自动在 `/dev/` 下创建设备文件 `/dev/mydev`

## 设备抽象

为了统一设备接口, 内核会将设备归类 (block, char, net), 然后提供通用接口.
然后将可用设备挂载到 `/dev` 目录下, 如 `/dev/sda, /dev/ttyUSB0, eth0`. 换言之,
这些设备虽然对用户可见, 但其驱动差异已经被屏蔽掉.
