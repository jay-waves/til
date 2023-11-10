### 变量类型
1. wire
2. reg
3. 向量 != 数组
- 向量:
各种类型都可以为向量, 声明例如`reg [8:2] data`. 特别的, 指定位宽的固定向量域访问`[bit+:width] or [bit-:width]`.
- 数组:
生命例如`reg [8:2] data[10]`, 或`integer data[8]`
1. parameter
2. data
	- integer: 有符号数(reg存无符号)
	- real: 十进制`1.37` 或 科学计数法`3e45`, 赋值给integer时会截断
3. time
	可调用系统函数`$time`来赋值