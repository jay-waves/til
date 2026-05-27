> Hint for anybody on the arm list: look at the dirstatthat Russell King posted, and if your ‘arch/arm/{mach,plat}-xyzzy’ shows up a lot, it’s quite possible that I won’t be pulling your tree unless the reason it shows up a lot is because it has a lot of code removed. People need to realize that the endless amounts of new pointless platform code is a problem, and since my only recourse is to say ‘if you don’t seem to try to make an effort to fix it, I won’t pull from you’, that is what I’ll eventually be doing.
> 
> 
> -- Linux Torvalds, 2011


Linux ARM 引入 Flattened Device Tree (FDT) 概念, 来用树形结构描述硬件. 用描述文件, 而不是驱动代码的方式来描述扳级硬件, 从而减少内核代码冗余. 

1. 在 DTS (Device Tree Source) `.dts` 文件描述硬件 (ARM 开发板), 放在 `arch/arm/boot/dts` 下. SoC 描述为 `.dtsi`, 然后由其他子 `dts` 引入.
2. 由 .. 编译 `.dts` 文件
3. 编译后的文件, 通过 Bootloader 传递给 Kernel. 

## DTS

`.dts` 文件包含 Node 和 Poperty, 以及根节点 `/`

```dts
/ {
	node 1 {
		a-string-property = "A string";
		a-string-list-property = "first string", "second string";
		a-byte-data-property = [0x01 0x23 0x34 0x56];
		child-node1 {
			first-child-property;
			second-child-property = <1>;
			a-string-property = "Hello, World";
		};
	};
}
```

### cpu

`cpu.reg` 指定了处理器的寄存器地址.

```dts
	cpus {
		#address-cells = <1>;
		#size-cells = <0>;

		cpu@0 {
			compatible = "arm,arm1176jzf-s";
			device_type = "cpu";
			reg = <0>;
		};
	};
```

### memory 

`memory.reg` 指定了内存范围和大小.

```dts
	memory@80000000 {
		device_type = "memory";
		reg = <0x80000000 0>;
	};
```

### spi 

`spi.reg` 指定了 SPI 控制器的地址范围和 Flash 设备的地址范围.

```dst
	spi1: spi@1e630000 {
		reg = <0x1e630000 0xc4>, <0x30000000 0x08000000>;
		#address-cells = <1>;
		#size-cells = <0>;
		compatible = "aspeed,ast2500-spi";
		clocks = <&syscon ASPEED_CLK_AHB>;
		status = "disabled";
		flash@0 {
			...
		};
		flash@1 {
			reg = <1>;
			compatible = "jedec,spi-nor";
			spi-max-frequency = <50000000>; /* freq: 50MHz */
			spi-rx-bus-width = <2>;
			status = "disabled"
		};
	};
```

### gpio

`gpio-cells` 指定 GPIO 数量

```dst
	gpio: gpio@1e780000 {
		#gpio-cells = <2>;
		gpio-controller;
		compatible = "aspped,ast2500-gpio";
		reg = <0x1e780000 0x200>;
		interrupts = <20>;
		gpio-ranges = <&pinctrl 0 0 232>;
		clocks = <&syscon ASPEED_CLK_APB>;
		interrupt-controller;
		#interrupt-cells = <2>;
	};
```

### timer 

```dst
	timer: timer@1e782000 {
		compatible = "aspeed,ast2400-timer";
		reg = <0x1e782000 0x90>;
		interrupts = <16 17 18 35 37 38 39>;
		clocks = <&syscon ASPEED_CLK_APB>;
		clock-names = "PCLK";
	};
```

### uart 

```dst
	uart1: serial@1e783000 {
		compatible: "ns16550a";
		reg = <0x1e783000 0x20>;
		reg-shift = <2>;  
		interrupts = <9>;
		clocks = <&syscon ASPEED_CLK_GATE_UART1CLK>;
		resets = <&lpc_reset 4>;
		no-loopback-test;
		status = "disabled";
	};
```

## DTB 

`.dts` 经过 DTC (Device Tree Compiler) 编译后生成二进制格式 `.dtb`. `.dtb` 一般被烧录在 ROM 的一个区域中, bootloader 装载时将其拷贝到内存, 供内核 (zImage) 使用.

DTC 源码位于 `scripts/dtc` 中. 