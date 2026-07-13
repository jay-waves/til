#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Analytic and numerical IK],
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

= Chapter 6: Inverse Kinematics

== Problem

Given a desired end-effector configuration $T_d$, inverse kinematics (IK) solves

$ T(theta) = T_d $

for the joint vector $theta$.  Unlike forward kinematics, the inverse map is
generally neither unique nor globally smooth:

- $T_d$ may lie outside the reachable workspace, so no exact solution exists.
- Several disconnected branches may reach the same pose, such as elbow-up and
  elbow-down configurations.
- Redundant robots may have infinitely many solutions.
- At a singularity, nearby task-space motions may require unbounded or
  nonexistent joint changes.

Joint limits, collision constraints, and branch continuity are part of the IK
problem, not merely checks to apply after a mathematical root is found.

== Analytic IK

Analytic IK converts the kinematic equations into a finite sequence of
trigonometric and algebraic operations.  It is fast, returns identifiable
solution branches, and needs no initial guess, but depends strongly on the
geometry of the mechanism.

For a planar 2R arm,

$ x &= L_1 cos theta_1 + L_2 cos(theta_1+theta_2), \
  y &= L_1 sin theta_1 + L_2 sin(theta_1+theta_2). $

The law of cosines gives

$ c_2 = (x^2+y^2-L_1^2-L_2^2)/(2L_1L_2), quad
  s_2 = plus.minus sqrt(1-c_2^2), $

$ theta_2=atan2(s_2,c_2), quad
  theta_1=atan2(y,x)-atan2(L_2s_2,L_1+L_2c_2). $

The sign of $s_2$ selects the elbow branch.  A solution exists only when
$abs(c_2)<=1$ up to numerical tolerance.

=== Decoupled Six-Axis Arms

A common 6R design has three wrist axes intersecting at one point.  Its IK can
be separated:

1. Use the desired orientation and the fixed tool offset to locate the wrist
   center.
2. Solve the first three joints as a position problem, producing shoulder and
   elbow branches.
3. Remove the orientation contributed by the first three joints and solve the
   remaining wrist rotation, producing wrist-flip branches.
4. Normalize angles, enforce limits, evaluate forward kinematics, and retain
   valid solutions.

PUMA-type arms exploit an intersecting spherical wrist.  Stanford-type arms
combine revolute positioning joints with a prismatic joint, but use the same
principle of isolating simple geometric subproblems.  Analytic formulas need
special handling where an $atan2$ pair vanishes or wrist angles become
singular.

== Pose Error on SE(3)

Subtracting homogeneous transformation matrices does not produce a meaningful
rigid-body displacement.  Use the group logarithm.  A body-frame error is

$ T_("err") = T(theta)^(-1) T_d $

$ [cal(V)_b] = log(T_("err")), $

while the equivalent space-frame error is

$ T_("err") = T_d T(theta)^(-1) $

$ [cal(V)_s] = log(T_("err")) $

The two error twists are related by the adjoint.  Pair a body error with $J_b$
and a space error with $J_s$; mixing them gives a frame-inconsistent update.

Write $cal(V)=(omega,v)$.  Separate stopping tolerances
$norm(omega)<=epsilon_omega$ and $norm(v)<=epsilon_v$ are preferable because
rotation and translation use different units.  A weighted norm can combine
them only after selecting a meaningful characteristic length.

== Newton-Raphson IK

For a scalar system $x=f(theta)$, Newton's method solves the local linearization
$x_d-f(theta) approx J(theta)Delta theta$.  On $"SE"(3)$, the group logarithm
provides the analogous local error.  With body coordinates,

$ cal(V)_b approx J_b(theta) Delta theta $

and the least-squares update is

$ Delta theta = J_b(theta)^dagger cal(V)_b $

$ theta arrow theta + Delta theta $

The linearization comes from a right perturbation of the pose:

$ T(theta+Delta theta)
  approx T(theta)e^([J_b(theta)Delta theta]). $

Consequently,

$ T(theta+Delta theta)^(-1)T_d
  approx e^(-[J_b Delta theta])T(theta)^(-1)T_d. $

Keeping first-order Lie-algebra terms gives

$ log(T(theta+Delta theta)^(-1)T_d)^("vee")
  approx cal(V)_b-J_b Delta theta. $

Setting the linearized residual to zero yields
$J_b Delta theta=cal(V)_b$ and therefore the pseudoinverse update.

A robust numerical procedure is:

1. Compute $T(theta)$ and $cal(V)_b=log(T(theta)^(-1)T_d)^("vee")$.
2. Stop successfully when angular and linear errors meet their tolerances.
3. Compute $J_b(theta)$ and solve $J_b Delta theta approx cal(V)_b$.
4. Limit or line-search the step, update $theta$, and normalize revolute joints
   only when their limits permit wrapping.
5. Stop with failure if the iteration limit is reached, the step becomes tiny
   while the error remains large, or the residual repeatedly fails to decrease.

Convergence is local and depends strongly on the initial guess.  Different
seeds can converge to different branches or fail even when a solution exists.
Warm-starting from the previous solution is effective for a continuous pose
trajectory.

== Damped Least Squares

The pseudoinverse magnifies error along directions with small singular values.
Damped least squares solves

$ argmin_(Delta theta) norm(J Delta theta-cal(V))^2
  + lambda^2 norm(Delta theta)^2 $

with solution

$ Delta theta = J^T (J J^T + lambda^2 I)^(-1) cal(V) $

Damping trades exact local error reduction for bounded joint steps.  Fixed
$lambda$ is simple but slows convergence everywhere; adaptive damping can stay
small in well-conditioned regions and grow as $sigma_("min")(J)$ decreases.

The damped equation is the stationary condition of

$ min_(Delta theta)
  norm(J Delta theta-cal(V))^2+lambda^2 norm(Delta theta)^2. $

Differentiating with respect to $Delta theta$ gives

$ (J^T J+lambda^2 I)Delta theta=J^T cal(V). $

The identity
$(J^T J+lambda^2 I)^(-1)J^T
=J^T(J J^T+lambda^2 I)^(-1)$ produces the displayed right-inverse form.

Step-size control is a separate mechanism.  Using
$theta arrow theta+alpha Delta theta$ with $0<alpha<=1$ and accepting a step
only when the pose error decreases improves global behavior.

== Redundancy and Constraints

For a redundant robot, the general differential update is

$ Delta theta = J^dagger cal(V)
  + (I-J^dagger J) eta. $

The second term is a first-order null-space motion.  Choosing
$eta=-k nabla h(theta)$ can reduce a secondary cost $h$, such as distance to a
preferred posture or a joint-limit barrier, without changing the primary task
to first order.

Null-space projection alone does not guarantee finite-step feasibility.
Practical constrained IK may instead solve a bounded least-squares or quadratic
program with joint position and step limits.  Collision avoidance requires
additional distance constraints or costs and a collision model.

#note[Clipping an unconstrained update at joint limits changes the direction of
the step and can destroy convergence.  Active-set or bounded solvers account
for the constrained directions while computing the step.]

== Inverse Velocity Kinematics

Inverse velocity kinematics solves the instantaneous problem

$ J(theta) dot(theta) = cal(V)_d $

Minimum-norm solution:

$ dot(theta) = J^dagger cal(V)_d $

This is not the same as finite-pose IK: integrating a desired twist open loop
accumulates modeling and numerical errors.  For trajectory tracking, add pose
feedback, for example

$ cal(V)_("cmd") = cal(V)_d + K cal(V)_("err"), quad
  dot(theta)=J^dagger cal(V)_("cmd"). $

The feedforward twist, error twist, and Jacobian must all use consistent space
or body coordinates.

== Closed-Loop Connection

The closure equations of a closed-chain mechanism are themselves inverse
kinematics constraints: multiple serial paths must produce the same body pose.
One may choose a subset of joint variables, solve the loop-closure equations
for the others, and differentiate the equations to obtain velocity constraints.
Closed chains are treated systematically in the next chapter.

== Takeaway

Analytic IK exposes exact solution branches when mechanism geometry permits.
Numerical IK is a local Newton iteration on a frame-consistent rigid-body pose
error, with the Jacobian, damping, step control, and constraints determining its
robustness in practice.
