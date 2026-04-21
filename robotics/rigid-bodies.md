at its most basic level a robot consists of rigid bodies connected by joints, with the joints driven by actuators

## configuration space of rigid bodies 

描述一个刚体的姿态的最小 $n$ 个实数，被称为*自由度（degrees of freedom）*。描述为一个向量，向量空间被称为 *configuration space (C-space)*.

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



## motion 

