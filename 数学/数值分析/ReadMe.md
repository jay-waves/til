如何指导计算机完成数值计算?
- 算法离散化
- 算法适用条件?
- 算法的计算量和存储量
- 误差


## 误差

误差 $e=x-a$

误差界 (误差限): $|e|\leq \epsilon$

相对误差: $e_{r}=\frac{e}{x}$

### 范数

**向量范数**是在 $\mathbb{R}^{n}$ 上的实值函数, 用于定义向量的大小:
1. 正定性: $\Vert x\Vert \geq 0$, 当且仅当 $x=0$ 时, $\Vert x\Vert =0$
2. 齐次性: 任意数 $k\in \mathbb{R}$, 有 $\Vert kx\Vert =|k|\cdot \Vert x\Vert$
3. 三角不等式: $\Vert x+y\Vert \leq \Vert x\Vert + \Vert y\Vert$.

对于 $x\in \mathbb{R}^n$, 有:

$$\Vert x\Vert_{1}=\sum^{n}_{i=1}|x_{i}|$$

欧几里得范数:
$$\Vert x\Vert_{2}=\sqrt{ \sum^{n}_{i=1}x^{2}_{i} }$$

无穷范数:
$$\Vert x\Vert_{\infty}=\max_{1\leq i\leq n}|x_{i}|$$

**矩阵范数**: 
1. 正定性
2. 齐次性
3. $\Vert A+B\Vert \leq \Vert A\Vert+\Vert B\Vert$
4. $\Vert AB\Vert \leq \Vert A\Vert \Vert B\Vert$

**矩阵范数需要与某种向量范数相容**, 即: ${} \Vert Ax\Vert \leq \Vert A\Vert\cdot \Vert x\Vert {}$.

### 数值稳定性

#### 1. 避免舍入误差传播

#### 2. 避免小数加到大数

浮点数加法不是严格结合律的. 求和时, 先将小数累加, 再加大数. 或者使用 Kahan 补偿求和.

#### 3. 避免两个近似值相减

$x=a+\epsilon_{0}$, $y=b+\epsilon_{1}$

各自的相对误差:

$\delta x=\frac{\epsilon_{0}}{a}$, $\delta y=\frac{\epsilon_{1}}{b}$

如果 $a\approx b$, 直接相减时, 相对误差占比会放大:

$\delta (x-y)=\frac{\epsilon_{0}-\epsilon_{1}}{a-b}$

可以采取 代数变换 或者 泰勒级数展开 的方式, 避免相近数的减法. 

#### 4. 避免除数远小于被除数

## 方程组求解

高斯消元法, LU 分解, OR 分解, 乔莱斯基分解, QR 分解, 雅各比法, 共轭梯度法

## 特征值求解

奇异值分析

## 函数逼近

插值法, 快速傅里叶变换

### 函数求解

牛顿法, 二分法

## 偏微分方程

## 数值积分

## 最优化问题

线性规划, 非线性规划