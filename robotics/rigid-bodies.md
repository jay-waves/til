at its most basic level a robot consists of rigid bodies connected by joints, with the joints driven by actuators

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

## Rigid-Body Motion [^1]

设固定单位坐标系 $s$ ，移动刚体单位坐标系 $b$ ，以两者原点为首尾的向量 $\Vec{p}$ 。

忽略位移， $s$ 中向量可通过旋转矩阵 $R$ 变为 $b$ 中向量： $$\Vec{v}_{s}=R\Vec{v}_{b}$$

旋转只有三个自由度，即旋转矩阵 $R$ 满足正交性和单位性： $$R^{\top}R=I \quad \det R=1$$ 它定义了空间中的一种李群 $SO(3)$ ，也叫[旋转矩阵群](quaternion.md)。

这里的单位坐标系都是右手系，满足： $$\Vec{x}\times \Vec{y}=\Vec{z}$$ 


[^1]: 这里使用 Modern Robotics 中的表示法。

