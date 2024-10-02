## Jordan 标准型

$$\mathbf{J}=\begin{bmatrix}
J_{m_{1}}(\lambda_{1})  & 0 & \cdots  & 0 \\
0 & J_{m_{2}}(\lambda_{2})  & \cdots  & 0 \\
\vdots  &  & \ddots  &  \\
0 & 0 & \cdots  & J_{m_{k}}(\lambda_{k}) 
\end{bmatrix}$$

其中 $$J_{m_{i}}(\lambda_{i})=\begin{bmatrix}
\lambda_{i} & 1 &  &  \\
 & \ddots  & \ddots  &  \\
 &  & \ddots  & 1 \\
 &  &  & \lambda_{i}
\end{bmatrix}_{m_{i}\times m_{i}}$$

默认排列顺序为 $\lambda_{1}\geq \lambda_{2}\geq \cdots \geq \lambda_{k}$

任何矩阵都相似于一个 Jordan 标准型: $A=PJP^{-1}$

特征值 $\lambda_{i}$ 的代数重数 $\beta_{i}$: $\mathbf{J}$ 的对角线上 $\lambda_{i}$ 出现次数

特征值 $\lambda_{i}$ 的几何重数 $n_{i}$: 主对角元为 $\lambda_{i}$ 的 Jordan 块个数. 可相似对角化时, 所有 Jordan 块都必须为一阶矩阵, 整个 $\mathbf{J}$ 退化为对角矩阵. 

https://wuli.wiki/online/ltrJor.html

https://blog.csdn.net/Insomnia_X/article/details/128796288

