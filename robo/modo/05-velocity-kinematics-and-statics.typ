#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Jacobians, singularities, manipulability],
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

= Chapter 5: Velocity Kinematics and Statics

== Jacobian Meaning

The manipulator Jacobian is the differential model of the forward kinematics.
For $T(theta)$, it maps joint rates to the end-effector twist:

$ cal(V) = J(theta) dot(theta), quad J in RR^(6 times n). $

Each column is the instantaneous twist created by unit velocity at one joint,
with all other joints fixed.  Therefore the range of $J$ is the set of
instantaneously achievable end-effector twists, while $null(J)$ contains
internal joint motions that leave the end effector stationary to first order.

For a spatial twist ordered as $cal(V)=(omega,v)$, the upper three rows of $J$
describe angular velocity and the lower three describe linear velocity of the
chosen frame origin.  Both the frame and the point whose linear velocity is
reported matter.

== Space Jacobian

For home screw axes $cal(S)_i$:

$ J_s(theta) = mat(cal(S)_1, Ad_(e^([cal(S)_1] theta_1)) cal(S)_2, dots, Ad_(e^([cal(S)_1] theta_1) dots e^([cal(S)_(n-1)] theta_(n-1))) cal(S)_n) $

Equivalently, define $T_i=e^([cal(S)_1]theta_1)dots
e^([cal(S)_i]theta_i)$.  Then

$ J_(s 1)=cal(S)_1, quad J_(s i)=Ad_(T_(i-1))cal(S)_i. $

Earlier joints move later joint axes in the space frame, so column $i$ depends
only on $theta_1,dots,theta_(i-1)$, never on its own joint value or later joints.
This triangular dependency is a useful implementation check.

The formula follows by differentiating the POE.  Write
$T=T_1 dots T_n M$ with $T_i=e^([cal(S)_i]theta_i)$.  Since
$dot(T)_i T_i^(-1)=[cal(S)_i]dot(theta)_i$,

$ dot(T)T^(-1)
  =sum_(i=1)^n T_1 dots T_(i-1)
   [cal(S)_i]dot(theta)_i
   T_(i-1)^(-1)dots T_1^(-1). $

Using $T[cal(S)]T^(-1)=[Ad_T cal(S)]$ and taking vector coordinates gives
$cal(V)_s=sum_i J_(s i)dot(theta)_i$, hence the stated columns.

== Body Jacobian

For body screw axes $cal(B)_i$:

$ J_b(theta) = mat(Ad_(e^(-[cal(B)_n] theta_n) dots e^(-[cal(B)_2] theta_2)) cal(B)_1, dots, Ad_(e^(-[cal(B)_n] theta_n)) cal(B)_(n-1), cal(B)_n) $

Later joints affect how earlier axes are seen from the body frame.  Thus column
$i$ depends only on $theta_(i+1),dots,theta_n$, and the last column is always
$cal(B)_n$.

Right-trivializing the derivative instead gives

$ T^(-1)dot(T)
  =sum_(i=1)^n T_n^(-1)dots T_(i+1)^(-1)
   [cal(B)_i]dot(theta)_i
   T_(i+1)dots T_n, $

which produces the body columns through the same conjugation identity.

The recursive evaluation from the end of the chain is

$ J_(b n)=cal(B)_n, quad
  J_(b i)=Ad_(e^(-[cal(B)_(i+1)]theta_(i+1)) dots
  e^(-[cal(B)_n]theta_n))cal(B)_i. $

== Space-Body Relation

For end-effector pose $T_("sb")(theta)$:

$ J_s(theta) = Ad_(T_("sb")(theta)) J_b(theta) $

Therefore $J_b=Ad_(T_("sb")^(-1))J_s$ and both Jacobians have the same rank.
The body Jacobian is often convenient for pose-error feedback because
$T^(-1)T_d$ lives naturally in the body frame; the space Jacobian is natural
when the desired twist is specified in fixed world coordinates.

=== Geometric Interpretation

At any configuration, the columns of $J_s$ are the current joint screw axes
expressed in the fixed frame.  The columns of $J_b$ describe those same physical
axes in the end-effector frame.  A revolute column $(omega,v)$ represents a line
through $q$ satisfying $v=-omega times q$; a prismatic column has $omega=0$.

== Other Jacobian Conventions

Not every “Jacobian” in software is the $6 times n$ twist Jacobian:

- An *analytic Jacobian* differentiates a coordinate chart such as
  $(x,y,z,phi,theta,psi)$.  Euler-angle rates are not angular velocity and the
  chart can have representation singularities unrelated to the mechanism.
- A *position Jacobian* maps joint rates to the linear velocity of a selected
  point.  If that point has fixed body coordinates $r_b$, its space position is
  $p+r$ with $r=R r_b$, and $dot(p+r)=v_s-omega_s times (p+r)$ under the spatial
  twist convention.
- A Jacobian expressed at a different point changes its linear rows even if the
  orientation coordinates are unchanged.

#note[Always record the twist ordering, expression frame, and reference point
when passing a Jacobian between libraries.  Matching matrix dimensions alone
does not establish compatible conventions.]

== Statics

Let $cal(F)$ be the wrench applied by the environment to the end effector, in
the same coordinates as $cal(V)$.  Virtual power balance gives

$ tau^T dot(theta) = cal(F)^T cal(V) $

With $cal(V) = J dot(theta)$:

$ tau = J(theta)^T cal(F) $

The Jacobian transpose maps endpoint wrench to joint torque.  This is not an
inverse and remains meaningful at singularities.  With several external
wrenches, first express each wrench and its associated Jacobian consistently,
then sum $tau=sum_i J_i^T cal(F)_i$.

The transpose law is forced by virtual power.  Substituting
$cal(V)=J dot(theta)$ into
$tau^T dot(theta)=cal(F)^T cal(V)$ yields

$ tau^T dot(theta)=cal(F)^T J dot(theta)
  =(J^T cal(F))^T dot(theta). $

Because this holds for every virtual $dot(theta)$, $tau=J^T cal(F)$.

Static equilibrium including actuator torque, external wrench, gravity, and
other generalized loads requires their signed sum to vanish.  The equation
$tau=J^T cal(F)$ alone describes only the wrench-to-joint mapping.

== Singularities and Manipulability

A configuration is singular when the Jacobian loses rank relative to its
maximum attainable rank:

$ rank(J(theta)) < "maximum rank" $

For a six-dimensional task, a chain with fewer than six independent joints is
structurally unable to generate arbitrary twists, but it is not singular at
every pose; singularity is defined relative to the chain's generic maximum
rank.  Common geometric causes include coincident screw axes and alignments
that make one column a combination of others.

At a singularity:

- the range of $J$ shrinks, so some end-effector twist directions are lost;
- $null(J)$ grows, admitting an instantaneous self-motion;
- $null(J^T)$ grows, allowing some endpoint wrenches that require no joint
  torque to balance;
- inverse velocity solutions become ill-conditioned before rank is lost
  exactly.

For an open chain, singularities are often classified as an *interior*
singularity caused by axis alignment or a *boundary* singularity at the edge
of the reachable workspace.  This distinction is geometric, not merely
numerical.

=== Manipulability Ellipsoids

For unit joint-rate norm, $norm(dot(theta))=1$, the principal axes of the
velocity ellipsoid are determined by the singular value decomposition
$J=U Sigma V^T$.  The achievable principal twist directions are the columns of
$U$, with semiaxis lengths $sigma_i$.

When $J$ has full row rank, its boundary can be written

$ cal(V)^T (J J^T)^(-1) cal(V) = 1 $

To derive it, fix a desired twist $cal(V)$ and choose the minimum-norm joint
rate

$ dot(theta)^*=J^T(J J^T)^(-1)cal(V). $

Then

$ norm(dot(theta)^*)^2
  =cal(V)^T(J J^T)^(-1)J J^T(J J^T)^(-1)cal(V)
  =cal(V)^T(J J^T)^(-1)cal(V). $

Thus the image of the unit joint-rate sphere has the stated boundary.

For unit joint-torque norm and a full-row-rank $J$, the corresponding wrench
ellipsoid is

$ cal(F)^T (J J^T) cal(F) = 1 $

This follows directly from unit joint torque:

$ 1=norm(tau)^2=norm(J^T cal(F))^2
  =cal(F)^T J J^T cal(F). $

The force semiaxis in a principal direction is $1/sigma_i$: large velocity
capability implies low force capability in the same direction.  At rank loss,
use the SVD directly rather than the inverse formula.

Scalar summaries include

$ mu_1 = sqrt(det(J J^T)), quad
  mu_2 = sigma_("min") / sigma_("max"), quad
  mu_3 = sigma_("min"). $

$mu_1$ measures ellipsoid volume up to a constant, $mu_2$ measures isotropy,
and $mu_3$ measures distance from a velocity singularity.  These measures
depend on units and on the relative weighting of translation and rotation.

== Inverse Velocity Kinematics

Minimum-norm solution:

$ dot(theta) = J^dagger cal(V)_d $

Redundant solution:

$ dot(theta) = J^dagger cal(V)_d + (I - J^dagger J) eta $

The null-space term preserves the commanded task velocity to first order and
can optimize a secondary objective, such as joint-limit avoidance or
manipulability.  With scalar objective $h(theta)$, a common choice is
$eta=k nabla h$.

Near singularities, the Moore-Penrose pseudoinverse may demand excessive joint
rates.  Damped least squares uses

$ dot(theta) = J^T (J J^T + lambda^2 I)^(-1) cal(V)_d $

and trades exact tracking for bounded rates.  The damping $lambda$ should grow
as the smallest singular value approaches zero.

== Takeaway

The Jacobian is the local differential model connecting motion and force.
Its columns expose the current joint screws, its range and null spaces describe
mobility and self-motion, and its singular values quantify directional
capability and numerical conditioning.
