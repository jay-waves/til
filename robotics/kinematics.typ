
#import "../template.typ": tufte, meta

#show: tufte

#meta[
- subtitle: Kinematics
]

== forward kinematics

$vec(theta)=(theta_1,...,theta_n)-> T(vec(theta)) in S E(3)$



== the URDF Format

```xml
<?xml version="1.0"?>

<robot name="ur5">

<!-- KNIEMATIC PROPERTIES (JOINTS) -->
<joint name="world_joint" type="fixed">
	<parent link="world"/>
	<child link="base_link"/>
	<origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/>
</joint>

<joint name="joint1" type="continuous">
	<parent link="base_link"/>
	<child link="link1"/>
	<origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.089159"/>
	<axis xyz="0 0 1"/>
</joint>

<!-- INERTIAL PROPERTIES (LINKS) -->
<link name="world"/>
<link name="base_link">
	<inertial>
		<mass value="4.0"/>
		<origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
		<inertia ixx="0.0044", ixy="0.0" ixz="0.0"
				iyy="0.0044" iyz="0.0" izz="0.0072"/>
	</inertial>
</link>
<link name="link1">
	<inertial>
		<mass value="3.7"/>
		<origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
		<inertia ixx="0.01026", ixy="0.0" ixz="0.0"
				iyy="0.01026" iyz="0.0" izz="0.0066"/>
	</inertial>
</link>	
```

== velocity kinematics

== inverse kinematics

== closed-chain kinematics
