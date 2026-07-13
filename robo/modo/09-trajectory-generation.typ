#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Paths, time scaling, and constraints],
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

= Chapter 9: Trajectory Generation

Trajectory generation assumes the endpoints or geometric path are already
chosen and assigns a smooth motion through them.  A *path* is independent of
time; a *trajectory* specifies configuration, velocity, and often acceleration
as functions of time.  Feasibility must account for kinematics, actuator limits,
collisions, and the controller's ability to track the result.

== Path and Time Scaling

Separate geometry from timing:

$ theta(t) = theta(s(t)), quad s in [0, 1] $

Velocity and acceleration follow from the chain rule:

$ dot(theta) = theta'(s) dot(s) $

$ ddot(theta) = theta'(s) ddot(s) + theta''(s) dot(s)^2 $

The scaling must satisfy $s(0)=0$, $s(T)=1$, and normally $dot(s)>=0$ so the
path is not traversed backward.  A path can be collision-free but dynamically
infeasible; time scaling enforces joint velocity, acceleration, torque, or
contact constraints without changing its geometry.

For a scalar joint velocity limit $abs(dot(theta)_i)<=v_i^("max")$, the path
imposes

$ dot(s) <= v_i^("max") slash abs(theta_i'(s)) $

where $theta_i'(s) != 0$.  Acceleration and torque limits constrain both
$dot(s)^2$ and $ddot(s)$.

== Joint-Space Straight Line

$ theta(s) = theta_("start") + s (theta_("end") - theta_("start")) $

This is simple, stays inside a convex box of joint limits when both endpoints
are inside it, and requires only one scalar time scaling.  However, the
end-effector path is generally curved and can collide even when both endpoint
poses are collision-free.

For revolute joints without hard multi-turn requirements, choose the intended
angle branch before interpolation.  Blindly wrapping each sample can introduce
discontinuities.

== Cubic Time Scaling

For rest-to-rest motion over duration $T$, impose
$s(0)=0$, $s(T)=1$, $dot(s)(0)=dot(s)(T)=0$.  The unique cubic is

$ s(t) = 3 (t / T)^2 - 2 (t / T)^3 $

with

$ dot(s)(t)=6t slash T^2-6t^2 slash T^3, quad
  ddot(s)(t)=6 slash T^2-12t slash T^3. $

Its peak scaling speed is $3/(2T)$ at $t=T/2$, and the endpoint acceleration
magnitude is $6/T^2$.  Acceleration jumps when adjoining a stationary segment,
so the required jerk is impulsive in the ideal model.

To obtain the polynomial, set
$s(t)=a_0+a_1t+a_2t^2+a_3t^3$.  The conditions at $t=0$ give
$a_0=a_1=0$.  The conditions at $T$ reduce to

$ a_2T^2+a_3T^3=1, quad 2a_2T+3a_3T^2=0. $

Solving gives $a_2=3/T^2$ and $a_3=-2/T^3$.

== Quintic Time Scaling

Adding $ddot(s)(0)=ddot(s)(T)=0$ gives

$ s(t) = 10 (t / T)^3 - 15 (t / T)^4 + 6 (t / T)^5 $

It enforces zero endpoint velocity and acceleration, producing continuous
acceleration when joined to rest.  Jerk is finite but generally discontinuous
at segment boundaries.  Higher-order polynomials can impose more derivative
conditions, but may oscillate and are not automatically dynamically feasible.

For $s(t)=sum_(i=0)^5 a_i t^i$, the three conditions at zero give
$a_0=a_1=a_2=0$.  With $r=t/T$, write
$s=b_3r^3+b_4r^4+b_5r^5$.  The terminal conditions become

$ b_3+b_4+b_5 &=1, \
  3b_3+4b_4+5b_5 &=0, \
  6b_3+12b_4+20b_5 &=0. $

Solving yields $(b_3,b_4,b_5)=(10,-15,6)$.

For a joint-space straight line with displacement
$Delta theta=theta_("end")-theta_("start")$, choose $T$ large enough that every
component of $Delta theta dot(s)$ and $Delta theta ddot(s)$ respects its limit.

== Cartesian Screw Trajectory

$ T(s) = T_("start") exp(log(T_("start")^(-1) T_("end")) s) $

This follows a constant body screw from start to goal.  Apart from the time
scaling, the body twist direction is constant.  It couples rotation and
translation according to the screw geometry, so the origin does not generally
move along a Euclidean straight line.

The matrix logarithm has branch choices near rotations of $pi$; select the
branch consistent with the intended motion and verify interpolation continuity.

== Cartesian Straight-Line Trajectory

$ R(s) = R_("start") exp(log(R_("start")^T R_("end")) s) $

$ p(s) = p_("start") + s (p_("end") - p_("start")) $

This preserves straight translation while following the shortest selected
geodesic in orientation.  It is distinct from screw interpolation.  Both are
task-space paths and require IK at each sample; IK branch jumps, unreachable
intermediate poses, joint limits, and collisions can make them infeasible even
when the endpoint poses are valid.

#note[Interpolate rotations on $"SO"(3)$, not by linearly interpolating rotation
matrix entries or Euler angles.  The latter can leave $"SO"(3)$ or encounter
coordinate singularities.]

== Polynomial Via-Point Trajectories

Via points specify intermediate joint configurations at times $t_i$.  Separate
polynomials on each interval offer local control.  Coefficients are determined
by waypoint values and continuity constraints:

- $C^0$: position is continuous;
- $C^1$: position and velocity are continuous;
- $C^2$: position, velocity, and acceleration are continuous.

Cubic splines commonly enforce $C^2$ continuity by solving for waypoint
velocities or accelerations.  Extra boundary conditions, such as zero endpoint
velocity, close the linear system.  Stopping at every waypoint is simple but
slow and creates unnecessary acceleration; passing through requires coordinated
derivatives on adjacent segments.

Polynomial interpolation does not enforce joint, collision, or torque limits by
itself.  Sample extrema analytically or densely, then lengthen segment times or
solve a constrained optimization.

== Time-Optimal Time Scaling

Substitute the path derivatives into the robot dynamics:

$ tau = M(theta(s))(theta'(s)ddot(s)+theta''(s)dot(s)^2)
  + c(theta(s),theta'(s)dot(s))+g(theta(s)). $

Because velocity-product terms are quadratic in velocity, each actuator torque
can be collected as

$ tau_i = a_i(s) ddot(s) + b_i(s) dot(s)^2 + c_i(s). $

Bounds $tau_i^("min")<=tau_i<=tau_i^("max")$ define lower and upper feasible
accelerations

$ L(s,dot(s)) <= ddot(s) <= U(s,dot(s)). $

For each actuator, define
$d_i=tau_i^("min")-b_i dot(s)^2-c_i$ and
$u_i=tau_i^("max")-b_i dot(s)^2-c_i$.  If $a_i>0$,

$ d_i/a_i<=ddot(s)<=u_i/a_i; $

if $a_i<0$, division reverses the inequalities.  Intersecting all actuator
intervals gives

$ L=max_i L_i, quad U=min_i U_i. $

Feasibility is exactly $L<=U$.

=== Phase-Plane View

In the $(s,dot(s))$ plane, feasibility requires $dot(s)>=0$ and $L<=U$.  The
curve where $L=U$ is the maximum-velocity curve; above it no acceleration is
admissible.  Since $ddot(s)=dot(s) d dot(s) slash d s$, acceleration bounds define
slopes of trajectories in this plane.

A classical time-optimal construction is bang-bang:

1. Integrate forward from $(0,dot(s)_0)$ using maximum acceleration $U$.
2. Integrate backward from $(1,dot(s)_T)$ using minimum acceleration $L$.
3. Switch where the profiles meet, adding switches along the maximum-velocity
   curve when required.
4. Recover time from $d t=d s slash dot(s)$.

The optimal scaling generally alternates between active torque bounds.  Special
points where $a_i(s)=0$, discontinuous path derivatives, or tangencies to the
maximum-velocity curve need careful numerical treatment.

== Takeaway

Trajectory generation separates geometric path design from time scaling.  The
chain rule connects them; polynomial scalings provide smooth point-to-point
motion, while phase-plane methods exploit actuator limits to traverse a fixed
path as quickly as permitted.
