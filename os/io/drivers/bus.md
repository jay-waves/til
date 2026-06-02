基于 Kernel2.6 

```c
struct device {
		struct klist klist_children;
		struct klist_node knode_parent; 
		struct klist_node knode_driver;
		struct klist_node knode_bus;
		struct device *parent;
		
		struct kobject kobj;
		char bus_id[BUS_ID_SIZE];
		
		struct bus_type* bus; // 设备所属的总线类型
		struct device_driver *driver;
		void *driver_data;
		void *platform_data; 
}
```

对象模型的如下图所示。设备和驱动都挂载在总线上，通过扫描 `bus->match()` 来绑定驱动和设备。

```ascii
struct bus_type
     |
     |  owns / indexes
     v
+-----------------------------+
| all devices registered here |
+-----------------------------+
     |
     +--> struct device dev0
     |       |
     |       +-- dev0.bus = &xxx_bus
     |       +-- dev0.driver = &drv_a   after matched
     |       +-- dev0.parent = parent device
     |
     +--> struct device dev1
             |
             +-- dev1.bus = &xxx_bus
             +-- dev1.driver = NULL     not bound yet
             +-- dev1.parent = parent device


struct bus_type
     |
     |  owns / indexes
     v
+-----------------------------+
| all drivers registered here |
+-----------------------------+
     |
     +--> struct device_driver drv_a
     |       |
     |       +-- drv_a.bus = &xxx_bus
     |       +-- drv_a.probe()
     |       +-- drv_a.remove()
     |
     +--> struct device_driver drv_b
             |
             +-- drv_b.bus = &xxx_bus
             +-- drv_b.probe()
             +-- drv_b.remove()
```