#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Error dynamics, motion control, force control],
  source: [Modern Robotics],
  revised: [2026-07-13],
  tags: ("robotics", "modern-robotics", "modo"),
)

#let argmin = math.op("arg min", limits: true)
#let atan2 = math.op("atan2")
#let rank = math.op("rank")
#let det = math.op("det")
#let tr = math.op("tr")
#let Ad = math.op("Ad")
#let ad = math.op("ad")
#let J = math.op("J")
#let null = math.op("null")
#let span = math.op("span")
#let range = math.op("range")
#let sign = math.op("sign")
#let sat = math.op("sat")

#let ddot(x) = math.dot.double(x)

= Chapter 11: Robot Control

Robot control closes the loop between a desired behavior, sensor measurements,
and actuator commands.  The available input matters: an industrial servo may
accept joint position or velocity, while a torque-controlled robot exposes the
rigid-body dynamics directly.  Contact tasks additionally require a model of
the environment and a clear choice between regulating motion, force, or their
dynamic relation.

== Error Dynamics

A controller shapes an error $e=x_d-x$ so it converges to zero.  The scalar
first-order system $dot(e)+k_p e=0$ has response $e(t)=e(0)e^(-k_p t)$ for
$k_p>0$.  A standard second-order target is

$ ddot(e) + K_d dot(e) + K_p e = 0 $

For scalar gains, comparison with
$ddot(e)+2 zeta omega_n dot(e)+omega_n^2 e=0$ gives
$k_p=omega_n^2$ and $k_d=2zeta omega_n$.  $omega_n$ sets nominal bandwidth and
$zeta$ determines overshoot and damping.  Critical damping corresponds to
$zeta=1$.

For multivariable errors, positive-definite gain matrices do not automatically
decouple the physical plant.  Sampling delay, filtering, actuator saturation,
unmodeled flexibility, and noise constrain usable bandwidth.  Integral action
can reject constant disturbances,

$ u=u_("ff")+K_p e+K_i integral e d t+K_d dot(e), $

but requires anti-windup when the actuator saturates.

== Joint Velocity Control

For desired trajectory $theta_d(t)$:

$ dot(theta) = dot(theta)_d + K_p (theta_d - theta) $

Then:

$ dot(e) + K_p e = 0 $

Indeed, $e=theta_d-theta$ implies

$ dot(e)=dot(theta)_d-dot(theta)
  =dot(theta)_d-(dot(theta)_d+K_p e)=-K_p e. $

For symmetric positive-definite $K_p$, the Lyapunov function
$V=1/2 e^T e$ satisfies
$dot(V)=-e^T K_p e<0$ for $e!=0$, proving exponential convergence.

This assumes the low-level velocity loop realizes the command accurately.
Velocity and acceleration limits should be applied with a reference governor or
trajectory filter; componentwise clipping changes the intended error dynamics.

For a multi-joint robot, diagonal $K_p$ is simple.  Coupled gains can shape
directions in joint space but should remain compatible with units and actuator
limits.

== Task-Space Velocity Control

For $T$ and $T_d$ in $"SE"(3)$, use a group-consistent pose error.  In body
coordinates,

$ [cal(V)_("err")] = log(T^(-1)T_d), $

and pair it with $J_b$.  A resolved-rate command is

$ dot(theta) = J^dagger (cal(V)_d + K_p cal(V)_("err")) $

The desired twist must be transformed consistently when the desired frame is
moving.  Space error with $J_s$ is an equivalent formulation, but mixing space
and body quantities is not.

For redundancy,

$ dot(theta)=J^dagger cal(V)_("cmd")+(I-J^dagger J)eta $

supports a secondary posture or joint-limit objective.  Near singularities,
damped least squares bounds joint speed at the cost of task error.  Hard limits,
collision avoidance, and prioritization are better handled by a constrained
least-squares or quadratic program than by clipping the final command.

== Computed Torque Control

Robot dynamics:

$ tau = M(theta) ddot(theta) + c(theta, dot(theta)) + g(theta) $

Commanded acceleration:

$ ddot(theta)_("cmd") = ddot(theta)_d + K_d dot(e) + K_p e $

Torque:

$ tau = M(theta) ddot(theta)_("cmd") + c(theta, dot(theta)) + g(theta) $

With an exact model and no unmodeled wrench, substitution yields the selected
linear error dynamics.  A more general inverse-dynamics law uses nominal model
terms,

$ tau = hat(M)(theta)(ddot(theta)_d+K_d dot(e)+K_p e)
  + hat(c)(theta,dot(theta))+hat(g)(theta). $

Model mismatch acts as a disturbance filtered by feedback.  Higher gains improve
disturbance rejection only until noise, delay, saturation, friction, or flexible
modes dominate.

For the exact model, substitute the control law into
$M ddot(theta)+c+g=tau$:

$ M ddot(theta)=M(ddot(theta)_d+K_d dot(e)+K_p e). $

Since $M$ is invertible,

$ ddot(theta)=ddot(theta)_d+K_d dot(e)+K_p e. $

Using $ddot(e)=ddot(theta)_d-ddot(theta)$ gives

$ ddot(e)+K_d dot(e)+K_p e=0. $

Computed torque is feedforward plus feedback linearization, not merely gravity
compensation.  Gravity compensation alone uses $tau=hat(g)(theta)+tau_("fb")$;
it is less model-dependent but leaves inertia and velocity coupling in the
closed-loop plant.

=== Task-Space Torque Control

A desired task acceleration can be mapped through operational-space dynamics.
For full-row-rank $J$,

$ Lambda=(J M^(-1)J^T)^(-1), quad
  cal(F)_("cmd")=Lambda cal(A)_("cmd")+mu+p, $

where $mu$ and $p$ denote task-space velocity and gravity terms.  Joint torque
is $tau=J^T cal(F)_("cmd")$ plus a dynamically consistent null-space torque.
Singularities and lower-dimensional tasks require reduced or damped formulas.

The simpler command $tau=J^T(K_p e+K_d dot(e))$ is Jacobian-transpose control;
it does not cancel task-space inertia but can be analyzed using energy methods
for suitable pose errors.

== Force Control

Endpoint wrench maps to joint torque:

$ tau = J(theta)^T cal(F) $

Pure position control against a stiff environment can create large contact
forces from tiny pose error.  In a constrained direction, integral force
feedback is often needed to remove steady-state wrench error:

$ cal(F)_("cmd")=cal(F)_d+K_f(cal(F)_d-cal(F)_("meas"))
  +K_i integral(cal(F)_d-cal(F)_("meas"))d t. $

The commanded wrench becomes joint torque through $J^T$, with dynamics and
gravity compensation added as needed.  Force sensing must be transformed to the
control frame and compensated for tool weight, sensor bias, and inertial loads.
Force control assumes contact; before contact, the environment cannot supply a
reaction wrench and the robot needs an approach-motion policy.

== Hybrid Motion-Force Control

Contact constrains motion in some directions while allowing force regulation in
the complementary directions.  If $P$ projects onto allowable motion directions
and $I-P$ onto constraint-wrench directions, a conceptual hybrid command is

$ cal(F)_("cmd")=P cal(F)_("motion")+(I-P)cal(F)_("force"). $

The projectors must be constructed using a consistent metric and contact frame;
ordinary diagonal selection matrices work only when coordinates align with the
natural constraints.  *Natural constraints* arise from contact geometry, while
*artificial constraints* specify the task.  Together they should neither leave
an uncontrolled direction nor command incompatible motion and force along the
same direction.

For ideal equality constraint $A cal(V)=0$, allowable twists lie in $null(A)$ and
reaction wrenches lie in $range(A^T)$.  With Euclidean coordinates and full row
rank $A$, the orthogonal force projector is

$ P_f=A^T(A A^T)^(-1)A, $

and the complementary motion projector is $P_m=I-P_f$.  They satisfy
$P_f^2=P_f$, $P_m^2=P_m$, and $P_f P_m=0$, which explains the hybrid split.

Switching at contact can cause impact and integrator windup.  Approach velocity,
contact detection, force ramps, and mode-transition logic are part of the
controller.

== Impedance and Admittance Control

Impedance control specifies a desired dynamic relation between motion error and
external wrench:

$ M_d ddot(e) + B_d dot(e) + K_d e = cal(F)_("ext") $

It commands wrench or torque so that the robot behaves like a virtual mass,
damper, and spring.  Positive-definite $M_d$, $B_d$, and $K_d$ give a passive
nominal mechanical behavior; low stiffness improves compliance but reduces pose
accuracy under load.

For constant reference and zero external wrench, define stored energy

$ E=1/2 dot(e)^T M_d dot(e)+1/2 e^T K_d e. $

Multiplying the homogeneous impedance equation by $dot(e)^T$ gives

$ dot(E)=-dot(e)^T B_d dot(e)<=0. $

Thus damping dissipates energy and the equilibrium is stable; with external
wrench, $dot(E)=dot(e)^T cal(F)_("ext")-dot(e)^T B_d dot(e)$, exposing the passive
input-output pairing up to the chosen error sign convention.

Admittance control is the causal dual: measured external force drives a virtual
dynamics model whose output is a motion command,

$ M_d ddot(x)_("cmd")+B_d dot(x)_("cmd")+K_d(x_("cmd")-x_0)
  = cal(F)_("ext"). $

It is convenient on robots with stiff position/velocity inner loops.  Impedance
is natural with good torque control.  Discrete-time delay, noisy force
derivatives, saturation, and a stiff environment can destroy apparent passivity,
so gains must be validated with the actual inner loop and contact stiffness.

== Takeaway

Robot control combines feedback error shaping with model-based feedforward and
explicit handling of actuator and contact behavior.  Motion, force, hybrid, and
impedance controllers solve different physical problems; their frame
conventions, inner-loop assumptions, limits, and transition logic determine
whether the textbook law works on a real robot.
