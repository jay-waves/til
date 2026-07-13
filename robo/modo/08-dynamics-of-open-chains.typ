#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Lagrange, Newton-Euler, forward dynamics],
  source: [Modern Robotics],
  revised: [2026-07-13],
  tags: ("robotics", "modern-robotics", "modo"),
)

#let argmin = math.op("arg min", limits: true)
#let exp = math.op("exp")
#let log = math.op("log")
#let sin = math.op("sin")
#let cos = math.op("cos")
#let tan = math.op("tan")
#let atan2 = math.op("atan2")
#let rank = math.op("rank")
#let det = math.op("det")
#let tr = math.op("tr")
#let Ad = math.op("Ad")
#let ad = math.op("ad")
#let J = math.op("J")
#let diag = math.op("diag")
#let null = math.op("null")
#let span = math.op("span")
#let sign = math.op("sign")
#let sat = math.op("sat")
#let dot = math.op("dot")
#let ddot = math.op("ddot")

= Chapter 8: Dynamics of Open Chains

Dynamics relates the robot's motion to actuator effort, gravity, contact, and
inertia.  The same model supports simulation, feedforward control, actuator
sizing, and estimation.  Two complementary derivations are useful: the
Lagrangian formulation exposes energy structure, while recursive Newton-Euler
is computationally efficient.

== Lagrangian Form

$ L(theta, dot(theta)) = K(theta, dot(theta)) - P(theta) $

Euler-Lagrange equation:

$ d / d t (partial L / partial dot(theta)_i) - partial L / partial theta_i = tau_i $

Robot dynamics:

$ tau = M(theta) ddot(theta) + c(theta, dot(theta)) + g(theta) $

or:

$ tau = M(theta) ddot(theta) + C(theta, dot(theta)) dot(theta) + g(theta) $

With an end-effector wrench $cal(F)_("tip")$ applied by the robot to the
environment, one common sign convention is

$ tau = M ddot(theta) + c + g + J^T cal(F)_("tip"). $

If the wrench is instead defined as acting on the robot, its sign reverses.
Every implementation must state this convention explicitly.

For conservative gravity, $g(theta)=partial P slash partial theta$.  The vector
$c(theta,dot(theta))$ groups all terms quadratic in velocity; a matrix
$C(theta,dot(theta))$ satisfying $c=C dot(theta)$ is not unique.

Let

$ K=1/2 sum_(j,k) M_(j k)(theta)dot(theta)_j dot(theta)_k. $

Then

$ partial K slash partial dot(theta)_i
  =sum_j M_(i j)dot(theta)_j, $

$ d/(d t)(partial K slash partial dot(theta)_i)
  =sum_j M_(i j)ddot(theta)_j
   +sum_(j,k)(partial M_(i j) slash partial theta_k)
    dot(theta)_j dot(theta)_k, $

while

$ partial K slash partial theta_i
  =1/2 sum_(j,k)(partial M_(j k) slash partial theta_i)
   dot(theta)_j dot(theta)_k. $

Substitution into Euler-Lagrange yields

$ tau_i=sum_j M_(i j)ddot(theta)_j
  +sum_(j,k)Gamma_(i j k)dot(theta)_j dot(theta)_k+g_i, $

where

$ Gamma_(i j k)=1/2(
  partial M_(i j) slash partial theta_k
  +partial M_(i k) slash partial theta_j
  -partial M_(j k) slash partial theta_i). $

Thus $c_i=sum_(j,k)Gamma_(i j k)dot(theta)_j dot(theta)_k$.

== Mass Matrix

Kinetic energy is:

$ K = 1 / 2 dot(theta)^T M(theta) dot(theta) $

$M(theta)$ depends on configuration because joint motion moves each link's mass
and rotational inertia differently at different poses.  For independent
coordinates it is symmetric positive definite:

$ M=M^T, quad x^T M x > 0 " for " x != 0. $

A useful construction is to evaluate inverse dynamics with zero velocity,
zero gravity, zero tip wrench, and unit acceleration $e_i$.  The resulting
torque is column $i$ of $M$:

$ M_(:i)(theta) = "ID"(theta,0,e_i,0,0). $

The kinetic-energy metric explains why $M$ also defines a configuration-
dependent notion of distance in velocity space.  Off-diagonal entries encode
inertial coupling: acceleration at one joint can require torque at another.

For standard rigid-body coordinates, the identity

$ dot(M)-2C " is skew-symmetric" $

implies $dot(theta)^T(dot(M)-2C)dot(theta)=0$.  This energy property is central
to stability proofs, although it depends on selecting a compatible $C$.

== Velocity-Product and Gravity Terms

Velocity-product terms include Coriolis and centrifugal effects and vanish when
$dot(theta)=0$.  They can be recovered from inverse dynamics as

$ c(theta,dot(theta))="ID"(theta,dot(theta),0,0,0). $

Gravity remains at rest and can be computed by

$ g(theta)="ID"(theta,0,0,g_s,0), $

where the base acceleration is initialized with the negative gravitational
acceleration, according to the algorithm's convention.  Static gravity
compensation applies $tau=g(theta)$ when no other loads act.

== Rigid Body Newton-Euler Form

For a body frame at the center of mass and axes aligned with principal inertia,
the spatial inertia is

$ cal(G)=mat(I_3,0;0,m I_3), $

under the $(omega,v)$ twist ordering.  In a general body-fixed frame it includes
cross terms induced by the center-of-mass offset.  For spatial inertia
$cal(G)$, body twist $cal(V)$, and body acceleration $dot(cal(V))$,

$ cal(F) = cal(G) dot(cal(V)) - ad_(cal(V))^T cal(G) cal(V) $

is the rigid-body Newton-Euler equation.  The second term accounts for the
change of momentum coordinates in a rotating frame.  Under a frame change,
twists transform with the adjoint and wrenches with its inverse transpose, so
the inertia transforms by the corresponding congruence.

== Inverse Dynamics

Inverse dynamics computes required torque:

$ tau = "ID"(theta, dot(theta), ddot(theta), F_("tip")) $

Newton-Euler uses two recursions:

1. *Forward pass:* propagate each link twist and acceleration from base to tip,
   adding the joint velocity and acceleration along its screw axis.
2. Compute each link's inertial wrench
  $cal(F)_i=cal(G)_i dot(cal(V))_i-ad_(cal(V)_i)^T cal(G)_i cal(V)_i$.
3. *Backward pass:* propagate child wrenches from tip to base, add each link's
   inertial wrench and external loads, and project onto the joint screw:
   $tau_i=cal(F)_i^T cal(A)_i$.

All transforms, screws, velocities, accelerations, inertias, and wrenches in a
recursion step must use compatible link frames.  The algorithm costs $O(n)$
for an $n$-link serial chain and avoids symbolic differentiation of kinetic
energy.

Inverse dynamics computes the torque for a specified state and acceleration;
it does not integrate motion.  Useful special calls extract the closed-form
terms:

$ M_(:i) &= "ID"(theta,0,e_i,0,0), \
  c &= "ID"(theta,dot(theta),0,0,0), \
  g &= "ID"(theta,0,0,g_s,0), \
  J^T cal(F)_("tip") &= "ID"(theta,0,0,0,cal(F)_("tip")). $

== Forward Dynamics

$ ddot(theta) = M(theta)^(-1) (tau - c(theta, dot(theta)) - g(theta) - J(theta)^T F_("tip")) $

The formula follows by isolating acceleration.  Since kinetic energy is positive
for every nonzero generalized velocity, $M$ is positive-definite and invertible;
therefore the dynamics define a unique acceleration for a given state, input,
and external wrench.

== Task-Space Dynamics

Operational-space inertia:

$ Lambda(theta) = (J M^(-1) J^T)^(-1) $

This operational-space inertia maps task acceleration demand to endpoint wrench
after accounting for robot inertia.  It exists as written when the task
Jacobian has full row rank.  The dynamically consistent generalized inverse is

$ J_M^+ = M^(-1)J^T Lambda, $

which satisfies $J J_M^+=I$ and minimizes kinetic energy among joint
velocities realizing a task velocity.  Near task singularities, use damping or
a reduced task rather than directly inverting $J M^(-1) J^T$.

The full task-space dynamics also contains velocity and gravity terms; $Lambda$
alone is not a complete Cartesian equation of motion.

To derive $Lambda$, temporarily omit bias terms and apply task wrench
$cal(F)$, so $tau=J^T cal(F)$ and
$ddot(theta)=M^(-1)J^T cal(F)$.  The induced task acceleration is

$ cal(A)=J ddot(theta)=J M^(-1)J^T cal(F). $

Hence

$ cal(F)=(J M^(-1)J^T)^(-1)cal(A)=Lambda cal(A), $

and substitution back into joint acceleration gives
$ddot(theta)=M^(-1)J^T Lambda cal(A)=J_M^+ cal(A)$.

== Constrained Dynamics

For holonomic constraints $phi(theta)=0$, define
$A(theta)=partial phi slash partial theta$.  Feasible accelerations obey

$ A ddot(theta) + dot(A) dot(theta) = 0. $

Constraint forces enter the joint dynamics as $A^T lambda$:

$ M ddot(theta)+c+g=tau+A^T lambda. $

Solving these equations together gives both acceleration and the Lagrange
multipliers $lambda$.  Constraint forces do no work on feasible velocities
because $dot(theta)^T A^T lambda=0$.  Contact constraints additionally require
unilateral and friction conditions; an equality constraint alone cannot model
separation or slip.

The multiplier can be eliminated algebraically.  Let $r=tau-c-g$.  From the
dynamics,

$ ddot(theta)=M^(-1)(r+A^T lambda). $

Inserting this into the acceleration constraint gives

$ (A M^(-1)A^T)lambda
  =-dot(A)dot(theta)-A M^(-1) r. $

For independent constraints,

$ lambda=-(A M^(-1)A^T)^(-1)
  (dot(A)dot(theta)+A M^(-1) r). $

Substitution into $ddot(theta)$ yields the unconstrained acceleration projected
onto the constraint-consistent tangent acceleration plus the curvature term
from $dot(A)dot(theta)$.

== Actuation Effects

Gear ratio reflects motor inertia approximately as:

$ I_("apparent") = G^2 I_("motor") $

For ideal gearing with motor angle $theta_m=G theta$ and joint torque
$tau=G tau_m$, motor speed rises by $G$ and motor inertia reflected to the joint
rises by $G^2$.  High gearing increases available joint torque and can make the
link-side inertia less important, but also reflects rotor inertia and friction,
reduces backdrivability, and introduces transmission compliance or backlash.

A simple motor model is

$ tau_m=k_t i, quad v=k_e dot(theta)_m+R i+L dot(i). $

Current is therefore approximately proportional to torque when electrical
dynamics are fast.  Voltage limits impose a speed-dependent torque limit due to
back electromotive force.

Friction models commonly combine viscous and Coulomb terms,

$ tau_f=b dot(theta)+tau_c sign(dot(theta)), $

with a set-valued or smoothed model near zero velocity.  Real joints may also
show stiction, Stribeck behavior, temperature dependence, and hysteresis.

Joint or link flexibility introduces additional coordinates and resonant modes.
A lumped joint model uses a spring-damper torque
$tau_k=k(theta_m slash G-theta)+d(dot(theta)_m slash G-dot(theta))$; a rigid-
robot model cannot predict the resulting oscillation.

== Takeaway

Robot dynamics combines a positive-definite inertia matrix, velocity coupling,
gravity, external and constraint wrenches, and actuator/transmission effects.
Lagrangian structure explains the model; recursive Newton-Euler evaluates it
efficiently; careful conventions and parameter validation make it usable.
