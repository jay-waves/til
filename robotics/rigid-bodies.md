at its most basic level a robot consists of rigid bodies connected by joints, with the joints driven by actuators

$$
\newcommand{\Vec}[1]{\mathbf{#1}}
\newcommand{\R}{\mathbb{R}}
$$

## configuration space of rigid bodies 

描述一个刚体的姿态的最小 $n$ 个实数，被称为*自由度（degrees of freedom）*。描述为一个向量，向量空间被称为 *configuration space (C-space)*.

The **task space** is a space in which the robot’s task can be naturally expressed.

The **workspace** is a specification of the configurations that the end-effector
of the robot can reach. 


#### 独立刚体自由度

平面上的刚体有三个自由度： $(x, y, \theta)$ 分别对应 xy 方向的平移与绕垂直轴旋转。

空间中的刚体有六个自由度： $(x,y,z, roll,yaw,pitch)$

![pnd-botics|400](http://oss.jay-waves.cn/til/adam-lite.avif)

#### 刚体关节

![|400](http://oss.jay-waves.cn/til/robot-joints.avif)

* F: fixed joint. 完全刚性连接。0DoF
* R: revolute joint, hinge joint. 铰链关节。1DoF
* P: prismatic joint, linear, sliding joint. 滑动关节。1DoF
* H: helical joint, screw joint. 螺旋关节。1DoF
* C: cylindrical joint. 2DoF. 圆柱接头。
* U: universal joint, 2DoF. 万向接头，有两个正交的铰链接头组成。
* S: spherical joint, 3DoF. 球形接头。

设 $N$ 个用关节连接在一起的刚体，刚体自由度为 $m$ （如空间中刚体无约束自由度为 $m=6$ ），关节的自由度为 $f$ ，那么总体的自由度：

$$\begin{align}
dof&=m(N-1)-\sum^{J}_{i=1}(m-f_{i}) \\
&=m(N-1-J)+\sum^{J}_{i=1}(f_{i})
\end{align}$$

#### C-space Topology 

球面上的点： $S^{1}$

直线上的点： $E^{1}$ 或 $R^{1}$

平面： $R^{2}$

平面上的刚体： $R^{2}\times S^{1}$

双铰链机械臂： $S^{1}\times S^{1}=T^{2}$

## Rigid-Body Rotation [^1]

设固定单位坐标系 $s$ ，移动刚体单位坐标系 $b$ ，以两者原点为首尾的向量 $\Vec{p}$ 。

忽略位移， $b$ 中向量可通过单位旋转矩阵 $R$ 变为 $s$ 中向量： $$\Vec{v}_{s}=R\Vec{v}_{b}$$

反之：$$\Vec{v}_{b}=R^{-1}\Vec{v}_{s}=R^{\top}\Vec{v}_{s}$$

这里的单位坐标系都是右手系，满足： $$\Vec{x}\times \Vec{y}=\Vec{z}$$ 

### Angular Velocities 

设坐标系绕单位向量 $\Vec{w}$ 旋转 $\Delta \theta$ 耗时 $\Delta t$ ，记角速度为： $$w=\Vec{w}\dot\theta=\Vec{w}\frac{\Delta \theta}{\Delta t}$$

三个坐标方向的角速度可以表示为： 

$$\begin{align}
\dot{\Vec x} &=w\times \Vec{x} \\
\dot{\Vec y} &= w\times \Vec{y} \\ 
\dot{\Vec z} & = w\times \Vec{z}
\end{align}$$

![|300](http://oss.jay-waves.cn/til/angular-velocity.avif)

用固定坐标系 $s$ 表示*旋转变化量* $w$, 即 $w_{s}$ ，设从固定坐标系 $s$ 到刚体坐标系 $b$ 的旋转矩阵 $R$ ，此时角速度可表示为： $$\dot{R}=w_{s}\times R=[w_{s}]R$$

向量叉乘可以写为“左乘一个斜对称矩阵”的形式，类似：

$$\Vec{a}\times \Vec{b}=\begin{bmatrix}
a_{1}\\a_{2}\\a_{3}
\end{bmatrix}\times \Vec{b}=\begin{bmatrix}
0 & -a_{3}  & a_{2} \\
a_{3} & 0 & -a_{1} \\
-a_{2} & a_{1}  & 0
\end{bmatrix}\times b=[\Vec{a}]\Vec{b}\tag{1}\label{skew-symmetric}$$

因此，$w_{s}$ 可以表示 SO(3) 中的等价矩阵：$$[w_{s}]=\dot{R}R^{-1}=\dot{R}R^{\top}$$

在考虑刚体坐标系下的旋转变化量 $w_{b}$ ，满足：[^2]

$$w_{b}=R^{-1}w_{s}$$ 

对于任意旋转矩阵 $R$ 和向量 $\Vec{v}$ ，有： $$R[\Vec{v}]R^{-1}=R[\Vec{v}]R^{\top}=[R\Vec{v}]$$

因此： $$[w_{b}]=[R^{-1}w_{s}]\dot{=}R^{-1}[w_{s}]R=R^{-1}(\dot{R}R^{-1})R=R^{-1}\dot{R}$$

总结： $$\begin{align}
[w_{s}]&=\dot{R}R^{-1} \\
[w_{b}]&=R^{-1}\dot{R}
\end{align}$$

### Exponential Coordinates of Rotation

![|300](http://oss.jay-waves.cn/til/rigid-body-motion.avif)

再来考察线速度，参考[四元数推导过程](quaternion.md)，对于固定坐标系中的向量 $p\in\R^{3}$ ，设其旋转角速度向量为 $w=\Vec{w}\dot{\theta}$ ，其端点线速度为： 

$$\dot{p}=w\times p=[w]p$$ 

这是一个线性微分方程，其解是 $$p(t)=e^{[w]t}p(0)$$ 


设 $t$ 时刻时，向量 $p$ 旋转了 $\theta$ 角度，此时将指数 $e^{[\Vec{w}]\theta}$ 泰勒展开得到 Rodrigues' Formula：

### Rodrigues' Formula 

$\newcommand{W}{{[\Vec{w}]}}$

**Given vector $\Vec{w}\theta\in \mathbb{R}^{3}$, such that $\theta$ is any scalar and $\Vec{w}\in \mathbb{R}^{3}$ is a unit vector, the matrix exponential of $\W\theta=[\Vec{w}\theta]\in SO(3)$ is**

$$Rot(\Vec{w},\theta)=e^{\W\theta}=I+\sin \theta\W+(1-\cos \theta)\W^{2}\in SO(3)\tag{2}\label{rodrigues}$$

Subsituting the skew-symmetric matrix representation of $\W$ in $\eqref{skew-symmetric}$  into $\eqref{rodrigues}$ , we obtain: 

$$R-R^{\top}=e^{\W}-(e^\W)^{\top}=2\sin \theta\W$$

## Rigid-Body Motion and Twist

### Homogenous Transformation Matrices 

Represent the regid-body transofrmation by a homogeneous transformation matrix in $SE(3)$: 

$$T=\begin{bmatrix}
R & p \\
0 & 1
\end{bmatrix}$$

where $R\in SO(3)$ represents the orientation of the coordinate frame, and $p\in \R^{3}$ represents the position of its origin. 

> $R\in SO(3)$ 是纯旋转群，3DoF；$T\in SE(3)$ 是刚体运动群，还有平移运动，6DoF。

<br> 

Some properties of T: 

$$T^{-1}=\begin{bmatrix}
R^{\top} & -R^{\top}p \\
0 & 1
\end{bmatrix}$$

appling on vectors:  

$$T\begin{bmatrix}
x  \\ 1
\end{bmatrix}=\begin{bmatrix}
R & p \\ 0  & 1
\end{bmatrix}\begin{bmatrix}
x \\ 1
\end{bmatrix}=
\begin{bmatrix}
Rx+p  \\ 1
\end{bmatrix}$$

pre-multiply $\dot{T}$ by $T^{-1}$ : 

$$T^{-1}\dot{T}=\begin{bmatrix}
R^{\top} & -R^{\top}p \\
 & 1
\end{bmatrix}\begin{bmatrix}
\dot{R} & \dot{p} \\
0 & 0
\end{bmatrix}=\begin{bmatrix}
R^{\top}\dot{R} & R^{\top}\dot{p} \\
0 & 0
\end{bmatrix}=\begin{bmatrix}
[\omega_{b}]  & v_{b} \\
0 & 0
\end{bmatrix}$$

from $Rx_{b}=x_{s}$, we get: $R^{\top}\dot{p}=v_{b}$ 

<br>

pre-multiply ${} T^{-1}$ by ${} \dot{T}$ : 
$$T^{-1}\dot{T}=
\begin{bmatrix}
\dot{R} & \dot{p} \\
0 & 0
\end{bmatrix}\begin{bmatrix}
R^{\top} & -R^{\top}p \\
j & 1
\end{bmatrix}=\begin{bmatrix}
\dot{R}R^{\top} & \dot{p}-\dot{R}R^{\top}p\\
0 & 0
\end{bmatrix}=\begin{bmatrix}
[\omega_{s}]  & v_{s} \\
0 & 0
\end{bmatrix}$$

in which, $\dot{p}-\dot{R}R^{\top}p=\dot{p}-[w_{s}]p=\dot{p}-w_{s}\times p=v_{s}$ .

对于空间中刚体，描述其任意一点 $x$ 的运动状态的量，称为*速度场*： $\dot{x}=\omega_{s}\times x+v_{s}$ 。令 $x=p$ ，即代表 $s$ 坐标系下的 $b$ 刚体坐标系原点的速度场，必须有： ${} \dot{p}-w_{s}\times p=v_{s} {}$

### Twist

*Body Twists* ( Spatial Velocity in body frame): 

$$\mathcal{V}_{b}=\begin{bmatrix}
\omega_{b} \\ v_{b}
\end{bmatrix}\in \R^{6}$$

defining: 

$$T^{-1}\dot{T}=[\mathcal{V}_{b}]=\begin{bmatrix}
[\omega_{b}]  & v_{b} \\
0 & 0k
\end{bmatrix}\in SE(3)
$$

similarly, defining *spatial twist (spatial velocity in the space frame)*: 

$$\mathcal{V}_{s}=\begin{bmatrix}
\omega_{s} \\ v_{s}
\end{bmatrix}\in \R^{6}$$ 

$$[\mathcal{V}_{s}]=
\begin{bmatrix}
[\omega_{s}]  & v_{s} \\
0 & 0
\end{bmatrix}=
\dot{T}T^{-1}\in SE(3)$$

then: $$[\mathcal{V}_{s}]=T^{-1}[\mathcal{V}_{b}] T$$

### Adjoint representation of T

Given $T$, its adjoint representation is: 

$$[Ad_{T}]=\begin{bmatrix}
R & 0  \\
[p]R & R
\end{bmatrix}\in \R^{6\times 6}$$

we have adjoint map: $$[\mathcal{V}_{s}]=\begin{bmatrix}
R & 0 \\
[p]R & R
\end{bmatrix}[\mathcal{V}_{b}]=Ad_{T}(\mathcal{V}_{b})$$

which can be proven equivalent as: $$[\mathcal{V}_{s}]=T^{-1}[\mathcal{V}_{b}]T$$


#### Exponential Coordinates of Motion&Twist

## Wrench

#TODO


[^1]: 这里使用 Modern Robotics 中的表示法。

[^2]: 再次强调，这里 $R$ 是指从固定坐标轴到刚体坐标轴的旋转矩阵。