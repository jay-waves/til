#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Mobile robot kinematics, planning, and control],
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

= Chapter 13: Wheeled Mobile Robots

Wheeled mobile robots move on a plane but differ in the instantaneous body
velocities their wheels permit.  Wheel geometry produces Pfaffian velocity
constraints; these determine mobility, controllability, planning methods, and
feedback laws.  Ideal rolling models are useful locally but wheel slip and
terrain interaction dominate real odometry accuracy.

== Wheel Models

For a conventional wheel, ideal contact imposes:

- pure rolling along the wheel plane: tangential chassis velocity equals wheel
  radius times spin rate;
- no lateral slip: contact velocity normal to the wheel plane is zero.

Swedish or mecanum wheels have passive rollers, relaxing one tangential
constraint.  Casters add steering coordinates whose orientation evolves with
motion.  A fixed standard wheel contributes a lateral no-slip constraint that
cannot be overcome instantaneously.

== Wheel Constraints

Let the planar chassis configuration be $q=(phi,x,y)$ and write its body twist
$cal(V)_b=(omega,v_x,v_y)$.  Wheel constraints can be assembled as

$ A(q) dot(q) = 0 $

or, including wheel rates $u$,

$ F cal(V)_b = G u. $

Holonomic velocity constraints integrate to constraints on configuration and
reduce the dimension of reachable configuration space.  Nonholonomic
constraints restrict instantaneous velocity but do not generally reduce the
dimension of the reachable set: sequences of allowed motions can create a net
displacement in a direction that is instantaneously forbidden.

The degree of mobility is the dimension of allowable body twists; the degree of
steerability counts independently steerable wheel orientations.  Rank tests
must be applied to the assembled constraint matrices at the current geometry.

== Omnidirectional Robots

Planar body twist:

$ cal(V)_b = mat(omega_z; v_x; v_y) $

Wheel speeds:

$ u = H cal(V)_b $

Each row of $H$ depends on wheel radius, wheel position, drive direction, and
roller angle.  If $H$ has full column rank, at least three independent wheel
directions allow arbitrary planar body twist.  Given desired twist,
$u=H cal(V)_b$ is inverse velocity kinematics.  Given measured wheel rates, a
least-squares forward estimate is

$ cal(V)_b=H^dagger u. $

With more than three driven wheels, wheel rates must be compatible; manufacturing
error or uneven ground can make exact no-slip constraints inconsistent.
Weighted least squares accounts for different wheel noise, while the residual
can indicate slip.

Omnidirectional motion simplifies geometric planning because short planar pose
changes can be followed directly.  Wheel-speed and acceleration limits still
couple allowable chassis twists, and mecanum layouts often have direction-
dependent force capacity.

== Differential Drive

For wheel radius $r$ and half-width $l$:

$ v = r / 2 (dot(phi)_R + dot(phi)_L) $

$ omega = r / (2 l) (dot(phi)_R - dot(phi)_L) $

These equations follow from the two longitudinal rolling constraints.  The
right and left contact points lie at lateral offsets $+l$ and $-l$, so their
forward velocities are

$ v_R=v+l omega, quad v_L=v-l omega. $

Pure rolling gives $v_R=r dot(phi)_R$ and
$v_L=r dot(phi)_L$.  Adding and subtracting the two equations yields the stated
$v$ and $omega$.

The body velocity is $cal(V)_b=(omega,v,0)$ and world kinematics are

$ dot(x)=v cos theta, quad dot(y)=v sin theta, quad dot(theta)=omega. $

The no-sideways-slip constraint

$ -sin theta dot(x)+cos theta dot(y)=0 $

is nonholonomic.  A differential drive cannot translate sideways
instantaneously, but can reach a sideways-displaced pose through rotations and
forward/backward segments.

Special cases are useful checks: equal wheel rates give straight motion;
opposite wheel rates give rotation in place; one stationary wheel gives a turn
about that wheel contact.  Unequal radii or an incorrect track width create
systematic odometry curvature error.

== Car-Like Robot

For steering angle $phi$ and wheelbase $L$:

$ dot(x) = v cos theta $

$ dot(y) = v sin theta $

$ dot(theta) = v / L tan phi $

The curvature is $kappa=tan(phi)/L$, so a steering bound implies a minimum turn
radius $R_("min")=L slash tan(abs(phi_("max")))$.  The model is often called the
kinematic bicycle model: front and rear axle wheel pairs are replaced by single
virtual wheels.  For ideal steering, physical left and right wheel angles must
differ so their axes intersect at a common instantaneous center (Ackermann
geometry).

Unlike differential drive, a car cannot rotate in place.  If reverse motion is
allowed, shortest obstacle-free pose connections can be built from
Reeds-Shepp curves; with forward motion only, Dubins curves use maximum-curvature
arcs and straight segments.  These curves are local steering tools, not complete
collision-aware planners by themselves.

== Controllability

A driftless nonholonomic system has the form

$ dot(q)=sum_i g_i(q)u_i. $

The control vector fields $g_i$ span instantaneous velocities.  Alternating
small motions can additionally generate Lie-bracket directions

$ [g_i,g_j]=(partial g_j slash partial q)g_i
  -(partial g_i slash partial q)g_j. $

If the vector fields and their iterated Lie brackets span the tangent space, the
system is locally accessible; under the usual driftless symmetric-control
conditions this yields small-time local controllability.  For a unicycle, the
forward and rotation fields plus their bracket span forward, rotation, and
sideways displacement.

Explicitly,

$ g_1=mat(cos theta;sin theta;0), quad
  g_2=mat(0;0;1). $

Since $g_2$ differentiates $g_1$ with respect to $theta$,

$ [g_1,g_2]=mat(sin theta;-cos theta;0) $

under the bracket convention above.  The matrix
$mat(g_1,g_2,[g_1,g_2])$ has determinant $1$, so its columns span $RR^3$ at
every configuration even though $g_1$ and $g_2$ alone span only two
instantaneous directions.

Controllability does not mean every path is trackable.  A path must satisfy the
nonholonomic tangent constraint, and actuator bounds affect how quickly bracket
motions can be executed.

== Motion Planning and Feedback

Planning approaches include:

- transform a geometric path into a feasible one using admissible steering
  primitives;
- search a state lattice built from precomputed feasible motions;
- use RRT with forward simulation of controls for kinodynamic problems;
- exploit differential flatness where outputs such as planar position
  parameterize state and input away from singular cases.

For a unicycle tracking a reference, define pose error in the robot or reference
frame rather than subtracting world coordinates blindly.  A common body-frame
error is

$ mat(e_x;e_y;e_theta)=
  mat(cos theta,sin theta,0;-sin theta,cos theta,0;0,0,1)
  mat(x_d-x;y_d-y;theta_d-theta). $

Feedback can couple forward speed to $e_x$ and angular speed to heading and
lateral error.  There is no smooth time-invariant static feedback that
asymptotically stabilizes a nonholonomic unicycle to a pose under standard
assumptions; practical pose stabilization uses time-varying, discontinuous, or
hybrid feedback.  Trajectory tracking is easier when the reference maintains
nonzero forward speed.

== Odometry

Odometry integrates incremental body motion from wheel encoders.  For a
differential drive over a short interval,

$ Delta s=r/2(Delta phi_R+Delta phi_L), quad
  Delta theta=r/(2l)(Delta phi_R-Delta phi_L). $

Using the exact constant-curvature arc or midpoint heading is more accurate than
updating translation with only the old heading.  When $Delta theta$ is near
zero, use the straight-line limit to avoid division by a small number.

For a constant body twist over the interval, the exact pose increment is the
$"SE"(2)$ exponential.  With $Delta theta!=0$ and initial heading $theta$,

$ Delta x &= Delta s/Delta theta
  (sin(theta+Delta theta)-sin theta), \
  Delta y &= -Delta s/Delta theta
  (cos(theta+Delta theta)-cos theta). $

Taking $Delta theta arrow.r 0$ gives
$Delta x=Delta s cos theta$ and $Delta y=Delta s sin theta$.

== Mobile Manipulation

End-effector twist combines base and arm motion:

$ cal(V)_("ee") = J_("base") dot(q)_("base") + J_("arm") dot(theta) $

The combined Jacobian can be written

$ cal(V)_("ee")=mat(J_("base"),J_("arm"))
  mat(u_("base");dot(theta)). $

Base inputs are not always arbitrary configuration rates; a nonholonomic base
must be parameterized by admissible controls.  Redundancy can keep the arm away
from singularities and joint limits, maintain visibility, reduce base motion,
or improve balance and wrench capability.

Whole-body control solves a weighted or constrained inverse problem with arm
and base velocity, acceleration, collision, and actuator bounds.  Planning them
sequentially is simpler but can miss solutions that require coordinated motion.
Dynamic manipulation also couples arm acceleration and contact wrench to base
stability and wheel traction.

== Takeaway

Wheel geometry defines allowable instantaneous twists.  Omnidirectional bases
span the planar twist space directly; nonholonomic bases synthesize missing
directions through motion sequences.  Planning, feedback, odometry, and mobile
manipulation must all preserve these constraints while accounting for slip and
actuator limits.
