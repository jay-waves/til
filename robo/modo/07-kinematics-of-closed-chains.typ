#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Loop constraints and parallel mechanisms],
  source: [Modern Robotics],
  revised: [2026-07-13],
  tags: ("robotics", "modern-robotics", "modo"),
)

#let argmin = math.op("arg min", limits: true)
#let exp = math.op("exp")
#let log = math.op("log")
#let atan2 = math.op("atan2")
#let rank = math.op("rank")
#let det = math.op("det")
#let tr = math.op("tr")
#let Ad = math.op("Ad")
#let ad = math.op("ad")
#let J = math.op("J")
#let diag = math.op("diag")
#let null = math.op("null")
#let range = math.op("range")
#let span = math.op("span")
#let sign = math.op("sign")
#let sat = math.op("sat")

= Chapter 7: Kinematics of Closed Chains

Closed chains contain multiple kinematic paths between bodies.  Their joint
coordinates cannot vary independently: every path must predict the same pose
for the shared body.  Parallel manipulators are an important class, with a
moving platform connected to a fixed base by several limbs.

== Constraint Equations

Collect all joint variables, actuated and passive, in $theta in RR^n$.  Loop
closure can be written as

$ g(theta) = 0 $

with $g:RR^n arrow.r RR^k$.  Not every joint vector is physically valid; regular
solutions lie on an $(n-rank(partial g slash partial theta))$-dimensional
constraint manifold.

For a loop represented by transforms, closure may be expressed as

$ T_1(theta) T_2(theta) dots T_m(theta) = I, $

or by equating the end-body configurations computed along two different paths.
The matrix equation contains redundant scalar entries, so numerical solvers
should use a minimal pose error, independent geometric constraints, or a rank-
aware method rather than treating all 16 matrix entries as independent.

== Mobility

For mechanisms assembled from rigid links, a generic mobility estimate is the
Gruebler-Kutzbach count

$ m = d (N-1-J) + sum_(i=1)^J f_i, $

where $d=3$ for planar and $d=6$ for spatial motion, $N$ includes the ground
link, $J$ is the number of joints, and joint $i$ has freedom $f_i$.  This is a
generic count: special geometry can make constraints dependent and change the
actual mobility.  The rank of the differential constraints is the decisive
local test.

== Forward and Inverse Kinematics

For a serial chain, joint coordinates usually determine the end pose directly.
For a parallel mechanism, terminology is relative to the actuators:

- *Inverse kinematics:* given the platform pose, solve the actuator lengths or
  angles.  Each limb can often be solved independently.
- *Forward kinematics:* given actuator coordinates, solve the coupled closure
  equations for platform pose and passive joints.  Multiple assembly modes may
  exist.

For a Stewart-Gough platform, base attachment points $b_i$, platform-frame
points $p_i$, platform pose $(R,p)$, and leg lengths $l_i$ satisfy

$ norm(p + R p_i - b_i)^2 = l_i^2, quad i=1,dots,6. $

The inverse problem evaluates these six lengths directly.  The forward problem
solves six nonlinear equations in the platform pose and may have several real
solutions.

#note[Choosing the nearest numerical root does not by itself preserve assembly
mode.  Continuation should track the previous configuration and monitor
singularities between samples.]

== Velocity Constraints

Differentiating $g(theta)=0$ gives

$ A(theta) dot(theta) = 0 $

where:

$ A(theta) = (partial g) / (partial theta) $

Feasible joint velocities lie in $null(A)$.  This equation states that the
relative twist around every closed loop sums to zero.  It can be assembled by
writing the twist of the shared body along every limb and equating the results
in a common frame.

For the Stewart-Gough leg vector $d_i=p+R p_i-b_i$ and unit direction
$u_i=d_i slash norm(d_i)$,

$ dot(l)_i = u_i^T (dot(p) + omega times R p_i). $

Stacking the rows maps the platform twist to leg rates.  The moment term can
also be written $(R p_i times u_i)^T omega$, making each row a wrench-like line
coordinate.

== Actuated and Passive Joints

Partition:

$ theta = mat(theta_a; theta_p) $

Then:

$ A_a dot(theta)_a + A_p dot(theta)_p = 0 $

If $A_p$ is invertible:

$ dot(theta)_p = - A_p^(-1) A_a dot(theta)_a $

This is the derivative supplied by the implicit-function theorem.  If
$g(theta_a,theta_p)=0$ and $partial g slash partial theta_p=A_p$ is nonsingular,
then locally $theta_p=h(theta_a)$.  Differentiating
$g(theta_a,h(theta_a))=0$ gives

$ A_a+A_p partial h slash partial theta_a=0, $

so

$ partial h slash partial theta_a=-A_p^(-1)A_a. $

This local relation is the closed-chain analogue of a Jacobian.  It exists only
when the passive-coordinate block has full rank.  More generally, one can
parameterize $null(A)$ as

$ dot(theta) = N(theta) dot(q), $

where the independent generalized coordinates $q$ have dimension equal to the
local mobility.  Actuator selection is valid only if actuator rates determine
$dot(q)$ without ambiguity.

Constraint forces do no virtual work on feasible velocities.  Hence they lie
in $range(A^T)$: a constraint generalized force has the form
$tau_c=A^T lambda$, where $lambda$ contains Lagrange multipliers.

== Serial vs. Parallel Difficulty

Serial chains usually have easy forward kinematics and difficult inverse
kinematics.  Parallel mechanisms often reverse that pattern: the inverse
kinematics decomposes by limbs, while forward kinematics requires solving
coupled nonlinear closure equations.  Both can have several branches, and
branch names differ:

- an *inverse-kinematic mode* selects a limb posture for a specified platform
  pose;
- an *assembly mode* selects a platform pose for specified actuator values.

== Singularities

Partition a differential input-output relation as

$ A_x(theta) cal(V) + A_q(theta) dot(q) = 0, $

where $cal(V)$ is the platform twist and $dot(q)$ contains actuator rates.
Different rank losses have different physical consequences:

- *Inverse-kinematic or serial singularity:* $A_q$ loses rank.  Some actuator
  rate is ineffective or a desired platform velocity demands an unbounded
  actuator rate; this often occurs at a workspace boundary.
- *Forward-kinematic or parallel singularity:* $A_x$ loses rank.  The platform
  gains an uncontrollable instantaneous motion even with actuators locked, and
  stiffness in the associated direction collapses.
- *Combined singularity:* both blocks lose rank.
- *Constraint singularity:* the full loop-constraint Jacobian loses rank,
  changing the local mobility of the mechanism.

These classifications depend on the selected inputs and outputs.  The robust
procedure is to state the variables, form the relevant linear map, and inspect
its rank and null spaces.

Near singularity, condition numbers and smallest singular values are more
informative than a determinant.  Crossing a singular set can change assembly
mode or make a numerical closure solver jump branches.

== Structural Trade-Off

Parallel mechanisms place multiple limbs in parallel, often giving high
stiffness, high load capacity, and low moving inertia.  The cost is a smaller
and more complicated workspace, internal collisions, parameter sensitivity,
closure-solving complexity, and dangerous parallel singularities.

== Takeaway

Closed-chain kinematics is governed by loop-closure constraints.  Their
nonlinear roots determine assembly modes, while the ranks and null spaces of
their derivatives determine mobility, actuator effectiveness, constraint
forces, and singular behavior.
