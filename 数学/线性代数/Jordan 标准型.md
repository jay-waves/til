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

***

矩阵不一定能[相似对角化](矩阵相似.md), 但是一定能化为若当标准型. 当矩阵几何重数等于代数重数时, 若当标准型就是对角矩阵.

### 若当标准化

设矩阵 $A$, 要求若当标准型 $J$, 满足 $A=PJP^{-1}$.

**步骤一**: 用矩阵特征多项式 $\det(A-\lambda E)=0$, 计算所有特征值 $\lambda_{i}$ 及其代数重数.

**步骤二**: 对于每个特征值 $\lambda_{i}$ 计算满足 $(A-\lambda_{i}E)x=0$ 的特征空间 $\mathrm{x}$ 中线性无关向量, 其个数为 $\lambda_{i}$ 的几何重数.

**步骤三**: 如果所有 $\lambda_{i}$ 的几何重数皆等于代数重数, 那么矩阵 $A$ 是可相似对角化的. 

**步骤四**: 如果某些特征值 $\lambda_{i}$ 的几何重数不等于代数, 则不能完全相似对角化, 为了化为若当标准型 $J$, 需要找到其广义特征向量.
此时通过解广义特征方程 $$(A-\lambda_{i}E)^{k}\cdot\mathrm{x}=0$$, 来寻找广义线性无关特征向量 $v_1, v_2, \dots, v_k$. 广义特征向量满足*若当链*递推关系: $$\begin{cases}
(A - \lambda_{i} E)\cdot v_1 = 0 \\
(A - \lambda_{i} E)\cdot v_2 = v_{1} & \Longleftrightarrow (A-\lambda_{i}E)^{2}\cdot v_{2}=0 \\
(A - \lambda_{i} E)\cdot v_3 = v_{2}  & \Longleftrightarrow(A-\lambda_{i}E)^{3}\cdot v_{3}=0\\
\vdots \\
(A - \lambda_{i} E)\cdot v_k = v_{k-1}  &\Longleftrightarrow (A-\lambda_{i}E)^{k}\cdot v_{k}=0\\
\end{cases}$$
, 在实际操作中, 不断增大 $k$, 直到方程核空间不再增大, 或链终止 ($(A-\lambda_{i})^{k}=0$).

**步骤五**: 对于每个特征值 $\lambda_{i}$, 不妨设其代数重数为 $m$, 几何重数为 $g$. **每个几何重数对应了一个线性无关的特征向量, 以它为起点构造一条若当链, 链长度 $k$ 为其对应其若当块大小.** *可能有多种构造方式(??)*, 但链长度的总和需要等于特征值的代数重数 $m$, 链的数量不能超过几何重数 $g$. 
特征值 $\lambda_{i}$ 对应的若当标准型(一部分)形式如下:

$$J(\lambda_{i})=\begin{bmatrix}
J_{k1}(\lambda_{i})  \\
0 & J_{k2}(\lambda_{i}) \\
0 & 0 & J_{k3}(\lambda_{i}) \\
0 & 0 & 0 & \ddots
\end{bmatrix}$$

**正确性证明**:

设 $\lambda_{i}$ 的一个若当块为 $$J_{3}(\lambda_{i})=\begin{bmatrix}
\lambda_{i} & 1 \\
0 & \lambda_{i} & 1 \\
0 & 0 & \lambda_{i} 
\end{bmatrix}$$

对应若当链为 $v_{1}, v_{2}, v_{3}$. 

类比 [矩阵相似对角化](矩阵相似.md) 中的形式, 若当标准化的局部为 $$A\cdot P(\lambda_{i})=P(\lambda_{i})\cdot J_{3}(\lambda_{i})=\begin{pmatrix}
v_{1}  & v_{2}  & v_{3}
\end{pmatrix}\begin{pmatrix}
\lambda_{i} & 1 \\
0 & \lambda_{i} & 1 \\
0 & 0 & \lambda_{i} 
\end{pmatrix}=(\lambda_{i}v_{1},\ v_{1}+\lambda_{i}v_{2},\ v_{2}+\lambda_{i}v_{3})$$

对于矩阵第二列: $$Av_{2}=v_{1}+\lambda_{i}v_{2}\ 
\Longleftrightarrow\ (A-\lambda_{i}E)v_{2}=v_{1}\Longleftrightarrow (A-\lambda_{i}E)^{2}v_{2}=0$$