## 符号的定义

* $\omega$ 模拟信号（连续时间的）角频率，单位为 *弧度每秒* $rad/s$
* $\Omega$ 数字信号（离散时间的）归一化角频率
* $f$ 物理频率，满足 ${} \omega=2\pi f {}$
* $f_{s}$ 采样频率，见[信号采样一节](信号.md)
* $t$ 线性系统的时间

## 常见序列

*单位抽样序列*：

$$\delta(t)=\begin{cases}1&,t=0\\0&,t\neq 0\end{cases}$$

$$x(m)\delta(t-m)=\begin{cases}x(t)&,\ t=m\\0&,\ \dots\end{cases}$$

*单位阶跃序列：* 

$$u(t)=\begin{cases}1&,t\geq 0\\0&,t\lt 0\end{cases}$$

*矩形序列：* $$R_N(t)=\begin{cases}1&,N-1\geq t\geq 0\\0&,t\lt 0\end{cases}$$
