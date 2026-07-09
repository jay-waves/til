#import "../../template.typ": tufte, note
#show: tufte

#import "@preview/cetz:0.5.2"

#let dtheta = $dot(theta)$
#let ddtheta = $dot.double(theta)$

Motion Control 有两种目标：末端执行器位置 $X_d(t)$ 或关节位置 $theta_d(t)$ ，这里主要研究 $theta_d$

= Velocity Input 

速度 $dtheta$ 作为控制系统的直接输出。

== Feedforward Control 

Given a desired joint trajectory: $theta_d(t)$, choose the commanded velocity $dot(theta)(t)$ as:

$ dtheta(t) = dtheta_d (t) $

Since no feedback (sensor data) is needed, _position errors_ will accumulate over time under the feedforward control law.

== P Control (Feedback Control)

_Proportional controller, or P controller_ is defined as: 

$ dtheta(t) = K_p (theta_d (t) - theta(t)) = K_p theta_e (t) $ 

_control gain_ $K_p > 0$.

$ dtheta_e (t) = dtheta_d (t) - dtheta(t) $

== PI Control

PI Controler: 

$ dtheta(t) = K_p theta_e(t) + K_i integral^t_0 theta_e(t)d t $

== Feedforaward + Feedback Control

反馈控制的缺点是，必须要有一个误差，用于启动控制系统。
我们可以根据先验知识，设定一个基准值（前馈值），让控制系统基于该基准值进行控制。

$ dtheta(t) = dtheta_d (t) + K_p theta_e (t) + K_i integral^t_0 theta_e (t) d t $


= Torque & Force Input 

力矩 $tau$ 作为控制系统的直接输出。

#figure(
  image("../../attach/torque-control.webp", width: 50%),
  caption: [Modern Robotis, Fig 11.11]
)

= Task-Space Motion Control 

Let $X(t) in S E(3)$ be the configuration of the end-effector, 
$cal(V)_b (t)$ be the end-effetor twist in end-effector frame ${b}$, $[cal(V)_b]=X^(-1)dot(X)$. 

The desired motion is given by $X_d (t)$ and $cal(V)_d (t)$. 
$[A d_(X^-1 X_d)]cal(V)_d$ expresses the feedforward twist $V_d$ in the actual end-effector frame at $X$ ($X_(s b)$), 
rather than the desired end-effector frame $X_d$ ($X_(s d)$)

$ cal(V)_b (t) = [A d_(X^-1 X_d)] cal(V)_d (t) + K_p X_e (t) + K_i integral^t_0 X_e (t) d t $

, where $X_e$ is not simply $X_d (t) - X(t)$, but $ [X_e] = log(X^-1 X_d) $

#note[记得 $X$, $cal(V)$ 都是关于 $t$ 的函数，后续不再写 $(t)$ 了。]



