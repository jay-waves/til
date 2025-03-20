
## Device Drivers

- block: like ide.c, device_setup() in `drivers/block/genhd.c`
- char: character-bases devices, like ttys
- cdrom: CDROM
- pci
- scsi
- net
- sound

1. 块设备 (block device), 按块缓存传输.
2. 字符设备 (char device), 字节流传输.

|              | 字符设备               | 块设备             |
| ------------ | ---------------------- | ------------------ |
| 缓存机制     | 不缓存                 | 系统缓存           |
| 随机访问     | 不支持, 仅顺序访问     | 支持, 任意位置读写 |
| 常见设备类型 | 键盘, 鼠标 `/dev/input/mice/`, 串口 `/dev/ttyS0`, 终端 | 硬盘 `/dev/sda`, SSD, USB `/dev/sdb`                  |