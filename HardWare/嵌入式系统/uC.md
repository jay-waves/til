微处理器 (uC, MCU):
- 处理器
- 内存地址映射:
	- ROM 
	- RAM 
	- Memory Map 
	- GPIOA 
- 引脚:
	- VCC 供电
	- GND 接地
- 外设:
	- GPIO (General purpose Input Output), 输出模式将引脚电平设置为 "高/低", 输入模式读取引脚电平 "高/低"
	- UART, 通用异步收发器, 控制串行通信协议.
	- SPI, 串行外设接口, 控制同步通信协议. 在 MCU/SoC 内作为独立的外设控制器.
	- 

## 配置 GPIO 

假设 MCU 有 32b 物理地址空间, 那么实际的外设固件会被映射到不同的地址区域. 比如 ROM 从 0x08000000 开始, RAM 从 0x2000000 开始 (可能有 2MB). MCU 的一些寄存器也被映射到特定的地址空间, 通过访问这些地址, 相当于访问了寄存器. 而寄存器控制着 MCU 及其外设的行为, 由具体的硬件定义.

GPIO 外设控制着 MCU 上的一组引脚, 其行为又受 MCU 的某些寄存器控制. 

```c
// 固件通过一组 32b 寄存器来描述一个 GPIO 外设. 
struct gpio {
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
};

// 假设 GPIO 外设实例 GPIOA 起始 地址为 0x40020000
#define GPIOA ((struct gpio *) 0x40020000)

/*
	多个 GPIO 合称为 Banks (GPIOA, GPIOB, ...), 假设它们的存储空间相隔 1KB
*/
#define GPIO(bank) ((struct gpio*) (0x40020000 + 0x400 * (bank)))

/*
	为了给引脚编号, 使用两字节的 uint16_t 数据结构, 高字节存储组号, 低字节表示序号.
	
*/
#define PIN(bank, num) (((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

// 比如, 定义引脚 A3 - GPIOA pin 3
uint16_t pinA3 = PIN('A', 3);

// 假设寄存器中某两位控制了某引脚的模式
enum {GPIO_INPUT, GPIO_OUTPUT, GPIO_AF, GPIO_ANALOG};

static void gpio_set_mode(uint16_t pin, uint8_t mode) {
	struct gpio *gpio = GPIO(PINBANK(pin));
	uint8_t n = PINNO(pin);
	gpio->MODER &= ~(3U << (n * 2));      // 清除当前寄存器设置
	gpio->MODER |= (mode & 3) << (n * 2);
}

gpio_set_mode(pinA3, GPIO_OUTPUT)
```

## 向量表

RAM 架构中, ROM 区最前的位置有 "向量表", 包含各类 中断处理程序 的地址. 向量表一般作为各类程序入口, 比如启动函数地址. 

```c
/*
	noreturn 和 naked 意味着标准函数的进入和退出部分不会被编译器创建.
*/
__attribute__((naked, noreturn) void _reset(void) {
	for (;;) (void) 0; // infinite loop
}

extern void _estack(void); // defined in link.ld

/* 
	16 + 91 个没有返回也没有参数的函数的指针数组, 组成了中断向量表. 放到独立的 vectors 段.
	前两个中断程序分别是:
		- estack, 堆栈指针
		- reset, 固件入口
*/
__attribute__((section(".vectors"))) void (*tab[16+91])(void) = {
	_estack, _reset 
};
```

接下来处理链接和重定向, 指定各个区段的起始位置.

```ld
ENTRY(_reset);
MEMORY {
	flash(rx) : ORIGIN = 0x08000000, LENGTH = 2048K
	sram(rwx) : ORIGIN = 0x20000000, LENGTH = 192K
}
_estack = ORIGIN(sram) + LENGTH(sram); /* stack points to end of SRAM */

SECTIONS {
	.vectors  :  { KEEP(*(.vectors)) }  > flash
	.text     :  { *(.text*) }          > flash
	.rodata   :  { *(.rodata*) }        > flash 

	.data : {
		_sdata = .;  /* .data section start */
		*(.first_data)
		*(.data SORT(.data.*))
		_edata = .;  /* .data section end */
	} > sram AT > flash 
	_sidata = LOADADDR(.data);

	.bss : {
		_sbss = .;
		*(.bss SORT(.bss.*) COMMON)
		_ebss = .;
	} > sram

	. = ALIGN(8);
	_end = .;  /* for cmsis_gcc.h */
}
```

## Glossary

- 串口 (Serial Port) 是逐位传输数据的通信接口, 用于设备间串行数据通信. 常见串口通信标准有: RS232, RS485, UART.
- DMA (Direct Memory Access) 允许外设在不经过 CPU 参与的情况下直接访问内存, 从而加快数据传输速度.
- 中断 (Interrupt) 是处理器响应异步事件的机制, 使处理器暂停当前任务, 转而执行中断服务程序 (ISR).

## Reference 

https://github.com/cpq/bare-metal-programming-guide/blob/main/README_zh-CN.md