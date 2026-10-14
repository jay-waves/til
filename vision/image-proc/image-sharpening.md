---
source: https://zhuanlan.zhihu.com/p/162275458  
---

Sobel， Laplacian， 高通滤波，

# 图像锐化

图像模糊的原因:
- 时域: 受到平均或积分运算影响.
- 频域: 高频分量被衰减.

图像锐化技术 (image sharpening) 使图像边缘和轮廓更加清晰, 方法分为*微分法, 高通滤波法*两类, 微分法还能分为*梯度法, Sobel算子法, Laplace算子法*. 
也能用于**图像边缘提取, [图像分割](./image-segmentation.md)**.

图像锐化前, **被处理图像必须有较高信噪比**, 否则噪声会受到的增强比信号还强.

## 微分法

微分运算是求信号的变化率, **加强高频分量, 使图像轮廓清晰**. 希望通过导数运算使边缘和轮廓变清晰, 这就要求该导数运算是各向同性的 (旋转不变性), 可以证明偏导数的平方和运算具有此性质.

### 1 梯度法

对于图像 $f(x,y)$,在点 $(x,y)$ 处的梯度是一个矢量, 对于离散的数字图像[^1]，定义其梯度和幅度为: 

[^1]: 离散化后，用差分代替微分

\[ \nabla f_{i,j} = 
    \begin{bmatrix} 
        f_{i+1,j}-f_{i,j}\\ 
        f_{i,j+1}-f_{i,j} 
    \end{bmatrix}, \qquad 

|\nabla f_{i,j}| =
    \sqrt{
    (f_{i+1,j}-f_{i,j})^2+
    (f_{i,j+1}-f_{i,j})^2
    }
\]

其幅度值可以简化成 

\[
|\nabla f_{i,j}|
\approx
|f_{i,j}-f_{i+1,j}|
+
|f_{i,j}-f_{i,j+1}|
\]


| f(i, j) | f(i+1, j) |
| ---- | --- |
| f(i, j+1) | |

图像的最后一行(列)的梯度无法求得, 一般用前一行(列)的梯度代替. 

当梯度计算完之后, 可以根据需要生成不同的梯度增强图像,

#### 变体一

只显示灰度变化大的边缘轮廓, 灰度变化平缓的呈黑色.
  
\[
g(x,y)=|\nabla f(x,y)|
\]

#### 变体二

可以显示出非常明显的边缘轮廓, 又不会破坏原灰度变化平缓的背景.

\[
g(x,y)=
\begin{cases}
|\nabla f(x,y)|, & |\nabla f(x,y)|\ge thresh,\\
f(x,y), & \text{otherwise}.
\end{cases}
\]

#### 变体三

边缘轮廓用固定的灰度级 $L_G$ 表示.

\[
g(x,y)=
\begin{cases}
L_G, & |\nabla f(x,y)|\ge thresh,\\
f(x,y), & \text{otherwise}.
\end{cases}
\]

#### 变体四

背景用固定灰度级 $L_G$ 表示, 便于研究边缘灰度的变化.

\[
g(x,y)=
\begin{cases}
|\nabla f(x,y)|, & |\nabla f(x,y)|\ge thresh,\\
L_G, & \text{otherwise}.
\end{cases}
\]

#### 变体五

将背景和边缘用二值图像表示, 便于研究边缘所在位置.

\[
g(x,y)=
    \begin{cases}
    L_G, & |\nabla f(x,y)|\ge thresh,\\
    L_B, & \text{otherwise}.
    \end{cases}
\]

### 3 Sobel 算子

采用梯度微分锐化图像, 会让噪声, 条纹得到增强. Sobel 算子在一定程度上解决了这个问题: 

| i-1,j-1 | i-1,j | i-1,j+1 |
| ------- | ----- | ------- |
| i,j–1   | i,j   | i,j+1   |
| i+1,j-1 | i+1,j | i+1,j+1        |

\[
\begin{aligned}
\triangle_x f_{i,j}
&= f_{i-1,j-1}+2f_{i-1,j}+f_{i-1,j+1}\\
&\quad-f_{i+1,j-1}-2f_{i+1,j}-f_{i+1,j+1},\\
\triangle_y f_{i,j}
&= f_{i-1,j+1}+2f_{i,j+1}+f_{i+1,j+1}\\
&\quad-f_{i-1,j-1}-2f_{i,j-1}-f_{i+1,j-1}.
\end{aligned}
\]

从这个式子中, 可以得到两个性质:

- Sobel 引入了平均的因素, 因此对噪声有一定的平滑作用
- Sobel 算子的操作就是相隔两个行/列的差分, 所以边缘两侧元素的得到了增强, 因此边缘显得粗而亮.

Sobel算子是矢量, 可以表示为: 

\[
H_1=
    \begin{bmatrix}
    -1&0&1\\
    -2&0&2\\
    -1&0&1
    \end{bmatrix},
\qquad
H_2=
    \begin{bmatrix}
    -1&-2&-1\\
    0&0&0\\
    1&2&1
    \end{bmatrix}
\]

### 4 Laplace 算子 (二阶微分)

拉普拉斯运算是**各向同性**的线性运算. 拉普拉斯算子为: $\nabla ^2f=\frac{\partial ^2f}{\partial x^2}+\frac{\partial ^2f}{\partial y^2}$. Laplace算子是标量, 由于具有旋转不变性, 所以其边缘提取效果不依赖于边缘方向.

由Laplace算子锐化后的图像为: $g=f-k \nabla ^2f$, 其中$k$ 为扩散效应的系数.

$$
\begin{align}
\frac{\partial^2 f}{\partial x^2}
&= f(i+1,j)-2f(i,j)+f(i-1,j),\\
\frac{\partial^2 f}{\partial y^2}
&= f(i,j+1)-2f(i,j)+f(i,j-1).
\end{align}
$$

$$
\begin{align}
\nabla^2 f(i,j)
&= \frac{\partial^2 f}{\partial x^2}
 + \frac{\partial^2 f}{\partial y^2}\\
&= f(i+1,j)+f(i-1,j)
 +f(i,j+1)+f(i,j-1)-4f(i,j).
\end{align}
$$

即, 数字图像在 $(i,j)$ 点的拉普拉斯算子, 约等于该点灰度值减去邻域四点的平均值求得. 可以表示为:

$$H=\left[ \begin{matrix} 0&-1&0\\ -1&4&-1\\ 0&-1&0\\ \end{matrix} \right]$$

其他常见的 Laplace 算子:

$H=\left[ \begin{matrix} 0&-1&0\\ -1&5&-1\\ 0&-1&0\\ \end{matrix} \right]$  $H=\left[ \begin{matrix} -1&-1&-1\\ -1&8&-1\\ -1&-1&-1\\ \end{matrix} \right]$ 
$H=\left[ \begin{matrix} 1&-2&1\\ -2&5&-2\\ 1&-2&1\\ \end{matrix} \right]$

## 高通滤波

高通滤波法**采用高通滤波器让高频分量通过, 使图像的边缘和线条变得清晰**, 实现锐化.

高通滤波器而用空域法和频域法实现, 在空间域使用卷积方法, 模板如:

$H=\left[ \begin{matrix} 0&-1&0\\ -1&5&-1\\ 0&-1&0\\ \end{matrix} \right]$ 

$H=\left[ \begin{matrix} -1&-1&-1\\ -1&9&-1\\ -1&-1&-1\\ \end{matrix} \right]$

$H=\left[ \begin{matrix} 1&-2&1\\ -2&5&-2\\ 1&-2&1\\ \end{matrix} \right]$ 

可以看出和拉普拉斯算子基本相同.

为了能够让高频通过, $(i,j)$ 点与其四个邻域点应该满足可以过零的性质, 因为导数跨零, 原函数曲线不会是单调的, 必然会在零点处改变单调性, 产生高频效果.

# 边缘检测

锐化方法对噪声比较敏感.

## Marr 边缘检测（理解即可）

Marr 提出先使用高斯函数进行[图像平滑](./image-denoising.md), 然后使用二阶拉普拉斯算子来求零交叉点 (二阶导数的交叉零点, 相当于一阶导数的峰). 

$$\nabla^{2}g(x,y)=\nabla^{2}(G(x,y,\sigma)*f(x,y))=\nabla^{2}G(x,y,\sigma)*f(x,y)$$

其中 $G$ 为高斯函数: 

$$G=\frac{1}{\sqrt{2\pi}\sigma^{2}}\exp\left[ -\frac{1}{2\sigma^{2}}(x^{2}+y^{2}) \right]$$ 

高斯滤波 $\sigma$ 越大, 频域越瘦 (和时域相反), 滤波额能力越强.

$f$ 为原图像, $g$ 为平滑后图像: $$g(x,y)=G(x,y,\sigma)*f(x,y)$$

$\nabla^{2}G(x,y,\sigma)$ 为 *LOG* 边缘检测算子 (Laplacian of Guassian): 

$$
\nabla^2 G(x,y,\sigma) =
\frac{\partial^2 G}{\partial x^2} +
\frac{\partial^2 G}{\partial y^2}
=
\frac{1}{\pi\sigma^4}
\left(
\frac{x^2+y^2}{2\sigma^2}-1
\right)
e^{-\frac{x^2+y^2}{2\sigma^2}}
$$

Marr 算子反映了人的视觉特性, 对于视神经细胞, 其输出称为 *DOG* 算子: 

$$
DOG(x,y,\sigma_1,\sigma_2)
=
G(x,y,\sigma_1)-G(x,y,\sigma_2)
$$

当 $\sigma_{1}/\sigma_{2}=1.6$ 时, DOG 逼近 LOG 算子.

## Hough 变换

形状匹配技术

...

## Unsharp Masking 

反锐化掩膜
