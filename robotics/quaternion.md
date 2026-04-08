$$\newcommand{\zm}{\lvert z\rvert}$$
$$\newcommand{\RM}{\begin{bmatrix}
\cos \theta & -\sin \theta \\
\sin \theta & \cos \theta
\end{bmatrix}}$$
$$
\newcommand{\Trp}{^\mathsf{T}} % 转置矩阵
$$

## 复数

复数 $a+bi$ 同构映射于矩阵形式： $$\begin{bmatrix}
a & -b \\
b & a
\end{bmatrix}$$

如 $$i\longleftrightarrow \begin{pmatrix}0 & -1\\ 1 & 0\end{pmatrix}$$

记复平面 $z=a+bi$ 与 $\Re$ 的正方向夹角为 $\theta$ ，那么： $$\cos \theta=\frac{a}{\sqrt{ a^{2}+b^{2} }}$$ $$\sin \theta =\frac{b}{\sqrt{ a^{2}+b^{2}}}$$

记 $\zm=\sqrt{ a^{2}+b^{2} }$ ，复数的同构矩阵可表示为*缩放矩阵*与*[旋转矩阵](../../../math/calculus/复数.md)*的乘积：$$z=\zm \cdot I\cdot \RM=\begin{bmatrix}\zm  & 0 \\
0 & \zm
\end{bmatrix} \cdot \RM=\zm e^{i\theta}$$ 

因此，一个复数可以表达一次旋转与缩放的线性变换。

## 轴角式旋转

设经过原点的旋转轴 $\Vec{u}=(x,y,z)\Trp$ 满足 ${} \Norm{\Vec{u}}=1 {}$，给定向量 $\Vec{v}$ ，使其沿着旋转轴转动 $\theta$ 角度，得到 $\Vec{v}'$ 。这里使用右手系统来定义旋转正方向。

轴角旋转有三个[自由度](rigid-bodies.md)，一个表示 $\theta$ ，两个表示单位向量 $\Vec{v}$ （与任意两个坐标轴的夹角）

![|600](http://oss.jay-waves.cn/til/20260331231138727.avif)

将单位向量 $\Vec{v}$ 沿垂直于轴 $\Vec{u}$ 和平行于轴的两个方向分解为 $v_{1},v_{2}$ 。平行分量 $\Vec{v}_{1}$ 在旋转后保持不变，垂直分量旋转后变为： 

$$\Vec{v}_{2}'=\cos \theta \cdot \Vec{v}_{2}+\sin \theta \cdot (\Vec{u}\times \Vec{v_{2}})$$

由于 $$v_{2}\times u=v\times u$$ $$v_{1}=(u\cdot v)u,\ v_{2}=v-v_{1}$$

因此，旋转后的向量有*三维旋转公式*： $$\Vec{v}'=\cos\theta \cdot\Vec{v}+(1-\cos \theta)(\Vec{u\cdot v})\Vec{u}+\sin \theta(\Vec{u}\times\Vec{v})$$

## 四元数

定义四元数 $q\in \mathbb{H}$ 满足： $$q=a+bi+cj+dk=[a,\Vec{v}]$$

其中：$$a,b,c,d\in\R$$ $$i^{2}=j^{2}=k^{2}=ijk=-1$$ $$\Vec{v}=(b,c,d)\Trp$$

四元数的模长： $$\Norm{q}=\sqrt{ a^{2}+b^{2}+c^{2}+d^{2} }$$

纯四元数：$$q=[0,\ \Vec{x}]$$

四元数的共轭： $$\Conj{q}=[s,-\Vec{v}]$$

四元数的互乘不满足交换律， $\mathbb{H}$ 是一个[除环](../../../math/algebra/环/环.md)： $$ij=k\neq ji=-k$$

#### GraBmann Product 

$$
\newcommand{\red}[1]{\textcolor{red}{#1}}
\newcommand{\blue}[1]{\textcolor{blue}{#1}}
$$

$$\begin{align}q_{1}q_{2}= 
&(ae-(bf+cg+dh)) + \\
& (b\red{e}+\blue{a}f+ch-dg)i + \\
& (c\red{e}+\blue{a}g+df-bh)j + \\
& (d\red{e}+\blue{a}h + bg-cf)k
\end{align}$$

令向量 $\Vec{x}=(b,c,d)\Trp,\ \Vec{y}=(f,g,h)\Trp$ ，那么：

$$q_{1}=[a,\Vec x]$$ $$q_{2}=[e,\Vec{y}]$$

$$q_{1}q_{2}=[ae-\Vec{x}\Vec{y},e\Vec{x}+a\Vec{y}+\Vec{x}\times \Vec{y}]$$

$$q\ \Conj{q}=\Conj{q}\ q=[a^{2}+\Vec{v}^{2},0]$$

四元数的逆： $$q^{-1}q=1$$ $$q^{-1}=\frac{\Conj{q}}{\Norm{q}^{2}}$$

纯四元数的积： $$vu=[0,\Vec{v}]\cdot [0,\Vec{u}]=[-\Vec{v}\cdot\Vec{u},\Vec{v\times u}]$$

## 单位四元数与旋转

设四元数 $$q=\cos \theta+\sin \theta \ \Vec{u}=[\cos \theta,\sin \theta\ \Vec{u}]$$ 

其中 $\Vec{u}$ 是单位旋转轴向量，可以证明 $q$ 是一个单位四元数： $$|q|=1$$

设垂直于轴的四元数 ${v}_{2}=[0,\Vec{v_{2}}]$ ，垂直分量的旋转公式可以改写为： 

$$\begin{align} \\
v_{2}'&=qv_{2}\\
&=[\cos \theta,\sin \theta\ \Vec{u}][0,\Vec{v_{2}}] \\
&=[-\sin \theta\Vec{u}\Vec{v_{2}},\cos \theta \Vec{v_{2}}+\sin \theta \Vec{u\times v_{2}}] \\
&=[0,\Vec{v_{2}'}]
\end{align}$$

$$\newcommand{\Sin}{\sin\theta\,}\newcommand{\Cos}{\cos\theta\,}$$

$$\newcommand{\W}{\Cos\Vec{v_{2}} + \Sin(\Vec{u\times v_{2}})}$$ 


设 ${} \Vec{w}=\W {}$ ，右乘 $q^{-1}$ 得到：（由于是单位向量，$q^{-1}=\Conj{q}$）

$$\begin{align}
(qv_{2})q^{-1}&=[0,\W][\Cos,-\Sin\Vec{u}] \\
&=[\Sin \Vec{w}\cdot \Vec{u},\Cos \Vec{w}-\Sin (\Vec{w\times u})] \\
&=[0,\cos(2\theta)\Vec{v_{2}}+\sin(2\theta)(\Vec{u\times v_{2}})] \\
&=[0,\cos(2\theta)(\Vec{v}-\Vec{(uv)u})+\sin(2\theta)(\Vec{u\times v})]
\end{align}$$

其中

$$\begin{align}
\Vec{w}\times \Vec{u}&=(\W)\times \Vec{u} \\
&=\Cos v_{2}\times u+\Sin(u\times v_{2})\times u  \\
&=\Cos v_{2}\times u+\Sin v_{2}
\end{align}$$

对于平行分量 $v_{1}$ 有：

$$\begin{align}
qv_{1}&=[\Cos,\Sin \Vec{u}][0,\Vec{(uv)u}] \\
&=[-\Sin \Vec{uv},\Cos\Vec{(uv)u}]
\end{align}$$

$$\begin{align}
(qv_{1})q^{-1}&=[-\Sin \Vec{uv},\Cos\Vec{(uv)u}][\Cos,-\Sin\Vec{u}] \\
&=[0,(\Vec{uv})\Vec{u}]
\end{align}$$

令 $\phi=2\theta$ 于是*四元数形式的三维旋转公式为*：

$$\begin{align}
qvq^{-1}&=q(v_{1}+v_{2})q^{-1}\\&=qv_{1}q^{-1}+qv_{2}q^{-1}\\
&=[0,\cos(\phi)\Vec{v}+(1-\cos(\phi))\Vec{(uv)u}+\sin(\phi)(\Vec{u\times v})]
\end{align}$$

#### 四元数形式的三维旋转公式

任意向量 $\Vec{v}$ 绕单位轴向量 ${} \Vec{u} {}$ 旋转 $\phi$ 度后得到 $\Vec{v}'$ ，在四元数定义下有旋转公式： $$v'=qvq^{-1}=qv\Conj{q}$$

其中：
* $q=[\cos(\phi/2),\sin(\phi/2)\Vec{u}]$
* ${} v=[0,\Vec{v}] {}$

## 插值

#### Slerp 插值


#### 

## 四元数与李群

2D、3D 旋转矩阵都是[*正交矩阵（Orthogonal Matrix）*](../math/linalg/对称矩阵.md)，它们的行列式值为 $1$ ，代表着旋转（$-1$ 代表反射）。

李群（Special Orthogonal Group, SO）是讨论旋转的更一般情况。单位四元数同构于某种李群。

