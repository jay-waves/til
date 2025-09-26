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

Cortex-M 系列开发板:

| 厂商                    | 芯片           | 内核        | 说明             |
| ----------------------- | -------------- | ----------- | ---------------- |
| STMicroelectronics (ST) | STM32 Fx/Gx    | M0/M3/M4/M7 |                  |
| NXP                     | Kinetis, LPC   | M0+/M4/M33  |                  |
| Nordic                  | nFR51          |             |                  |
| TI                      | MSP432, CC13xx | M4          |                  |
| Microchip (Atmel)       | SAM D/E/F      | M0+/M4      | 主要用于 Arduino |
| Silicon Labs            | EFM32, EFR32   | M0+/M4/M33  |                  |

ARM MCU 厂商会提供 CMSIS 头文件, 对外设进行 API 抽象. 

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
#define BIT(x) (1UL << (x))
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
	RCC->AHB1ENR |= BIT(PINBANCK(pin));   // 使能 GPIO Clock
	gpio->MODER &= ~(3U << (n * 2));      // 清除当前寄存器设置
	gpio->MODER |= (mode & 3) << (n * 2);
}

static inline void gpio_wr(uint16_t pin, bool val) {
	struct gpio *gpio = GPIO(PINBANK(pin));
	gpio->BSRR = (1U << PINNO(pin)) << (val ? 0: 16);
}

// 延时函数
static inline void spin(volatile uint32_t count) {
	while (count--) asm("nop");
}

gpio_set_mode(pinA3, GPIO_OUTPUT)
```

STM32 的 GPIO 口有如下控制寄存器:
- MODER, 模式寄存器. 控制: 输入 (00), 输出 (01), 复用 (10), 模拟 (11)
- OTYPER, 输出类型寄存器: push-pull (0), open-drain (1)
- OSPEEDR, 输出速度寄存器
- PUPDR, 上拉/下拉寄存器: 00, 01, 10
- IDR, 输入数据寄存器. 一位对应一个引脚.
- ODR, 输出数据寄存器
- BSRR, 批量设置引脚
- LCKR: 配置锁存寄存器, 锁定某个 GPIO 的值, 直到下次复位
- `AFR[2]`: 复用功能寄存器, 用于模拟更高级外设功能

在 STM32 中, 一个 GPIO 端口通常有 16 个引脚. `GPIOx_PIN0 ~ GPIOx_PIN15`. GPIO 实际是挂载在 AHB/APB 总线上的外设, 需要时钟信号才能访问其寄存器. GPIO 等外设的时钟, 需要用 RCC (Reset and Clock Control) 中的 `AHBxENR` 寄存器来使能:

```c
RCC->AHB1ENR  |=  RCC_AHB1ENR_GPIOAEN; // 打开 GPIOA 的时钟, 否则读不出数据. 
```

GPIO 可以被其他外设复用, 通过复用器将其作为某个外设的信号线使用. 比如将 `AF7` 作为 USART Tx/Rx 使用, 同时脱离 ODR/IDR 控制. 

## 向量表

RAM 架构中, ROM 区最前的位置有 "向量表", 包含各类 中断处理程序 的地址. 向量表一般作为各类程序入口, 比如启动函数地址. 对于 ARM MCU, 向量表前 16 个地址作为保留, 其余向量作为外部中断程序的入口. 

MCU 加载固件程序时, 有两个值很重要:
- 初始栈指针
- 程序入口地址

### 最小固件

```c
/*
	noreturn 和 naked 意味着标准函数的进入和退出部分不会被编译器创建.
*/
__attribute__((naked, noreturn) void _reset(void) {
	extern long _sbss, _ebss, _sdata, _sidata;
	// copy .data to RAM 
	for (long *dst = &_sdata, *src = &_sidata; dst < &_edata) 
		*dst++ = *src++;
	// memset .bss to zero
	for (long *dst = &_sbss; dst < &_ebss; dst++) *dst = 0;
	
	main(); 
	for (;;) (void) 0; // infinite loop
}

extern void _estack(void); // defined in link.ld

/* 
	16 + 91 个没有返回也没有参数的函数的指针数组, 组成了中断向量表. 
	放到独立的 vectors 段. 两个中断程序分别是:
		- estack, 堆栈指针
		- reset, 固件入口
*/
__attribute__((section(".vectors"))) void (*tab[16+91])(void) = {
	0, _reset, 0, ...., 0, 0, SysTick_Handler
};

int main(void) { 
	return 0;
}
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

## 时间控制

ARM MCU 内置一个 SysTick 定时器, 通过 4 个寄存器控制:
- CTRL: 使能/禁能 SysTick 
- LOAD: 初始计数值
- VAL: 当前计数值, 每个时钟周期递减. 24b 有效
- CALIB: 校准寄存器

设时钟周期为 16MHz, SysTick 中断在向量表中索引为 15. 当 VAL 值递减为 0 时, 会产生一个 SysTick 中断. 

```c
struct systick {
	volatile uint32_t CTRL, LOAD, VAL, CALIB;
};

#define SYSTICK ((struct systick *) 0xe00e010) 
```

让 SysTick 每 1ms 产生精确中断:

```c
static inline void systick_init(uint32_t ticks) {
	if ((ticks -1) > 0xffffff) return; // 24b val 
	SYSTICK->LOAD = ticks - 1;
	SYSTICK->VAL = 0;
	SYSTICK->CTRL = BIT(0) | BIT(1) | BIT(2); // enable systick 
	RCC->APB2ENR |= BIT(14); // enable syscfg
}

systick_init(16000000 / 1000); // systick per ms

static volatile uint32_t s_ticks; 
void SysTick_Handler(void) {
	s_ticks++;
}
```

定时器工具如下. s_ticks 更新是即时的 (通过中断异步更新), 但是 timer_expired 的调用并不是即时, 这可能导致调用时机稍有延迟. 但是这个误差并不会漂移. 

```c
bool timer_expired(uint32_t *t, uint64_t prd, uint64_t now) {
	if (now + prd < *t) *t = 0;  // reset timer
	if (*t == 0) *t = now + prd; 
	if (*t > now) return false;
	*t = (now - *t) > prd ? now + prd : *t + prd;
	return true;
}

// 调用时
uint32_t timer = 0, period = 600;
if (timer_expired(&timer, period, s_ticks)) {
	...
}
```

## 串口调试

uart 知识详见 [外设与总线驱动](外设与总线驱动.md)

```c
struct uart {
	volatile uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR;
};

#define UART1 ((struct uart*) 0x40011000)
#define UART2 ((struct uart*) 0x40004400) 
#define UART3 ((struct uart*) 0x40004800)


// 配置 GPIO 复用
static inline void gpio_set_af(uint16_t pin, uint8_t af_num) {
	struct gpio *gpio = GPIO(PINBANK(pin)); 
	int n = PINNO(pin);
	gpio->AFR[n >> 3] &= ~(15UL << ((n & 7) * 4));
	gpio->AFR[n >> 3] |= ((uint32_t) af_num) << ((n & 7) * 4);
}

// 串口初始化
#define FREQ 16000000
static inline void uart_init(struct uart *uart, unsigned long baud) {
	uint8_t af = 7; // alternate function
	uint16_t rx = 0, tx = 0; // pins 
	if (uart == UART1) RCC->APB2ENR |= BIT(4);
	if (uart == UART2) RCC->APB1ENR |= BIT(17);
	if (uart == UART3) RCC->APB1ENR |= BIT(18);
	
	if (uart == UART1) tx = PIN('A', 0), rx = PIN('A', 10);
	if (uart == UART2) tx = PIN('A', 2), rx = PIN('A', 3);
	if (uart == UART3) tx = PIN('D', 8), rx = PIN('D', 9);
	
	// 设置串口时钟及复用
	gpio_set_mode(tx, GPIO_MODE_AF);
	gpio_set_af(tx, af);
	gpio_set_mode(rx, GPIO_MODE_AF);
	gpio_set_af(rx, af);
	
	uart->CR1 = 0; // disable this uart 
	// 设置波特率
	uart->BRR = FREQ / baud; 
	// 使能串口外设
	uart->CR1 |= BIT(13) | BIT(2) | BIT(3); // set UE, RE, TE
}

// 串口读写: 如果 RXNE 被使能, 说明数据可用
static inline int uart_read_ready(struct uart *uart) {
	return uart->SR & BIT(5); 
}

static inline uint8_t uart_read_byte(struct uart *uart) {
	return (uint8_t) (uart->DR & 255);
}

// 设置发送数据后, 等待数据完成
static inline void uart_write_byte(struct uart* uart, uint8_t B) {
	uart->DR = B;
	while ((uart->SR & BIT(7)) == 0) spin(1);
}

// 或者配置一个缓冲区
static inline void uart_write_buf(struct uart* uart, char *buf, size_t len) {
	while (len-- > 0) uart_write_byte(uart, *(uint8_t *) buf++);
}
```

用例: 
```c
uart_init(UART3, 115200); 
if (...)
	uart_write_buf(UART3, "hi\r\n", 4);
```

## Glossary

- 串口 (Serial Port) 是逐位传输数据的通信接口, 用于设备间串行数据通信. 常见串口通信标准有: RS232, RS485, UART.
- DMA (Direct Memory Access) 允许外设在不经过 CPU 参与的情况下直接访问内存, 从而加快数据传输速度.
- 中断 (Interrupt) 是处理器响应异步事件的机制, 使处理器暂停当前任务, 转而执行中断服务程序 (ISR).

## Reference 

https://github.com/cpq/bare-metal-programming-guide