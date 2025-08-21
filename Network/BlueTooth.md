蓝牙标准由 **Bluetooth SIG (Special Interest Group)** 维护. 主要版本如下:
- 2.0 + EDR, 2004. 引入增强数据率 (EDR, Enhanced Data Rate), 传输速度最高达 3Mbps
- 3.0 + HS, 2009. 引入高速传输 (HS)
- 4.0, 2010. 引入低功耗蓝牙 (BLE, Bluetooth Low Energy)
- 4.2, 2014. 支持 IPv6.
- 5.0, 2016. 

蓝牙有两种主要工作模式;
- BR/EDR: 传统蓝牙
- LE (Low Energy): 低功耗蓝牙, 一般用于传感器.

## 协议层次

蓝牙是一种多层次协议

- 应用层:
	- A2DP: 音频传输, 用于蓝牙耳机
	- HFP: 免提通话
	- HID: 键盘鼠标
	- GATT=based Profiles: 传感器, 心率 (HRP) / 电池 (BAS) / 温度 (HTP) 等
- 通用访问配置层: 
	- GAP (Generic Access Profile), 定义设备如何**广播, 扫描, 连接, 配对**
	- GATT (Generic Attribute Profile): BLE 的高层应用协议, 用于描述设备如何暴露服务.
- 属性层: ATT (Attribute Protocol), 定义设备属性
- 安全管理: SMP (Security Manager Protocol): 配对, 加密, 认证等功能. 密码协议使用 AES-CCM 
- 逻辑链路控制: L2CAP (Logical Link Control and Adaption Protocol), 
- 主机控制接口 HCI (Host Controller Interface), 主机和蓝牙控制器间的接口, 支持通过 UART/USB/SPI 等串口通信
- 控制器层: (数据链路层, 物理层等): 负责设备发

## 通信过程

1. 外设周期性广播
2. 中央设备 (Central) 扫描, 并发现外设存在
3. Central 发起连接请求 (GATT), 查询外设的服务和特征.
4. 数据交互