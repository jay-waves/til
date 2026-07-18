
#import "../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [机器人运动学],
  source: [Modern Robotics, Chapter 4-6],
  revised: [2026-07-13],
  tags: ("robotics", "modern-robotics"),
)

= forward kinematics 


== D-H

Denavit-Hartenberg form:

$ T_04 = T_01 T_(12) T_23 T_34 $

== PoE

#image("../attach/robo-PoE.webp", width: 70%)

#let screw(i) = $[cal(S)_#i]$
#let mscrewm(i) = $M^(-1) [cal(S)_#i] M$
#let body(i) = $[cal(B)_#i]$

== Space POE Formula

Set all joint variables to zero and record the home configuration
$M=T(0)$.  Express every joint screw axis in the fixed space frame at this same
home configuration.  Then

$ T(theta) = e^([cal(S)_1] theta_1)
  e^([cal(S)_2] theta_2) dots
  e^([cal(S)_n] theta_n) M. $

The exponentials appear in joint order from base to end effector.  Although
$cal(S)_i$ is recorded at home, the preceding exponentials automatically move
its physical axis to the current configuration.

This follows from conjugation.  If $T$ carries a screw coordinate frame to the
space frame, then

$ T e^([cal(S)]theta)T^(-1)
  =e^(T[cal(S)]T^(-1)theta)
  =e^(["Ad"_T cal(S)]theta). $

After joints $1,dots,i-1$ move, the current space screw of joint $i$ is

$ cal(S)_i(theta)= "Ad"_(e^([cal(S)_1]theta_1)dots
  e^([cal(S)_(i-1)]theta_(i-1)))cal(S)_i. $

Applying each joint motion to the chain already moved by its predecessors gives
the ordered product inductively.

== Body POE Formula

Alternatively, express every screw axis in the end-effector frame at home:

$ T(theta) = M e^([cal(B)_1] theta_1)
  e^([cal(B)_2] theta_2) dots
  e^([cal(B)_n] theta_n). $

The ordering of the joint variables is unchanged; only the side on which the
motion product acts is different.  Space and body descriptions are equivalent:

$ cal(S)_i = "Ad"_M cal(B)_i,
  quad cal(B)_i = "Ad"_(M^(-1)) cal(S)_i. $

To derive the body form, use the conjugation identity

$ e^([cal(S)_i]theta_i)M
  =M(M^(-1)e^([cal(S)_i]theta_i)M)
  =M e^(["Ad"_(M^(-1))cal(S)_i]theta_i). $

Repeatedly moving $M$ left through the space exponentials produces the body
POE and proves $cal(B)_i="Ad"_(M^(-1))cal(S)_i$.

For a body-frame construction, it is often easiest to imagine locking every
joint except joint $i$, moving $theta_i$ positively, and observing the
instantaneous screw of the end-effector frame expressed in ${b}$.
$vec(theta)=(theta_1,...,theta_n)-> T(vec(theta)) in S E(3)$

=== Space form of PoE

=== Body form of PoE

$
T(theta)
  &= e^(screw(1)theta_1) dot ... dot e^(screw(n)theta_n) M \
  &= e^(screw(1)theta_1) dot ... dot M e^(mscrewm(n)theta_n) \
  &= e^(screw(1)theta_1) dot ... dot M e^(mscrewm(n-1)theta_(n-1))
     e^(mscrewm(n)theta_n) \
  &= M e^(body(1)theta_1) dot ... dot e^(body(n-1)theta_(n-1))
     e^(body(n)theta_n)
$

$body(i) = mscrewm(i)$, i.e., $cal(B)_i = ["Ad"_M^(-1)]cal(S)_i$


== the URDF Format

The URDF (Universal Robot Description Format) is an XML file sued by ROS2 to describe
 the kinematics, inertial properties, and link geometry of robots

=== Joints

*Joints* connect two links: a _parent_ and a _child_ link 
  - types: prismatic, revolute, continuous (revolute without joint limits), fixed (virtual joints which doesnot permit any motion)
  - _origin frame_: defines the child link frame relative to the parent link frame *in zero position*.
  - _axis_: unit vector along the rotation axis in child link frame. 

```xml
<joint name="joint1" type="continuous">
  <parent link="base_link"/>
  <child link="link1"/>
  <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.089159"/>
  <axis xyz="0 0 1"/>
</joint>
```

=== Link

The joints describe the kinematics of a robot, the link define tis mass properteis.
- _origin frame_ describes the position and orientation of a frame at the link's center of mass relative to the link's joint frame
- _inertia matrix_ ... (inertia matrix is symmetric, it's only necessary to define the terms on and above the diagonal)

#note[Inertia Matrix 详见机器人动力学（dynamics of robots）]

```xml
<link name="world"/>

<link name="base_link"/>
  <inertial>
  <mass value="4.0"/>
  <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
  <inertia ixx="0.0044" ixy="0.0" ixz="0.0"
            iyy="0.044" iyz="0.0" izz="0.0072"/>
  </inertial>
</link>

<link name="link1"/>
  <inertial>
  <mass value="3.7"/>
  <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
  <inertia ixx="0.0102" ixy="0.0" ixz="0.0"
            iyy="0.0102" iyz="0.0" izz="0.0066"/>
  </inertial>
</link>
```

#note[URDF 不支持环结构]

== velocity kinematics

== inverse kinematics

== closed-chain kinematics
