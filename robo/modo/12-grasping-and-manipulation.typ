#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Contacts, closure, and manipulation],
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
#let range = math.op("range")
#let sign = math.op("sign")
#let sat = math.op("sat")
#let dot = math.op("dot")
#let ddot = math.op("ddot")

= Chapter 12: Grasping and Manipulation

Grasping studies how contacts restrict object motion and transmit wrench.
Manipulation additionally chooses and changes contacts to move an object.  The
analysis is local and mode-dependent: rolling, sliding, separation, and impact
obey different equations.

== Contact Kinematics

At a point contact, let $n$ point along the separating normal and let the contact
point velocities on the two bodies be $v_A$ and $v_B$.  The normal relative
velocity is

$ v_n=n^T(v_A-v_B). $

With a consistent sign convention, nonpenetration imposes $v_n>=0$ at a closed
contact.  Separation has $v_n>0$; maintained contact has $v_n=0$.  Contacts
therefore constrain relative body motion through inequalities such as

$ A cal(V) >= 0 $

where $cal(V)$ collects object or body twists.  At a maintained contact, the
active normal row becomes an equality.

The velocity of a body-fixed point $p$ under spatial twist
$cal(V)=(omega,v)$ is

$ dot(p)=omega times p+v. $

To derive it, write a material point as $p=p_o+R r$, where $r$ is constant in
the body frame.  Then

$ dot(p)=dot(p)_o+dot(R)r
  =dot(p)_o+[omega]R r
  =dot(p)_o+omega times (p-p_o). $

Under the spatial-twist convention $v=dot(p)_o-omega times p_o$, this reduces to
$dot(p)=omega times p+v$.

This converts contact geometry into linear rows acting on twists.  With several
bodies, stack all body twists and insert equal-and-opposite point-velocity terms
in the columns of the two bodies sharing each contact.

=== Contact Modes

- *Breaking:* positive separating normal velocity; normal force is zero.
- *Sliding:* zero normal velocity and nonzero tangential relative velocity;
  friction opposes slip at the boundary of the friction cone.
- *Sticking:* zero normal and tangential relative velocity; tangential force may
  lie inside the friction cone.
- *Rolling:* contact points have zero relative velocity while material points
  at the contact change; curvature determines higher-order evolution.

First-order constraints may miss motion enabled or blocked by surface
curvature.  A zero first-order normal velocity is therefore not always proof of
finite-motion feasibility.

== Form Closure

Form closure means unilateral contact geometry alone prevents every object
motion, without friction or actuator effort.  If the rows of $A$ encode allowed
normal separation, the first-order condition is

$ { cal(V) | A cal(V) >= 0 } = {0} $

Equivalently, there is no nonzero instantaneous twist satisfying all
nonpenetration constraints.  In wrench language, positive combinations of the
frictionless contact normals must positively span the wrench space.

Let the contact-normal wrench rows be $a_i^T$, so admissible motion satisfies
$a_i^T cal(V)>=0$.  If nonnegative contact magnitudes are $lambda_i$, their net
reaction wrench is

$ cal(F)=A^T lambda=sum_i lambda_i a_i, quad lambda_i>=0. $

The motion cone and reaction-wrench cone are polar because

$ cal(F)^T cal(V)=lambda^T A cal(V)>=0. $

The motion cone reduces to ${0}$ exactly when its polar is the entire wrench
space, which is the positive-spanning statement for first-order form closure.

Force magnitudes are unilateral: contacts push but do not pull.  Ordinary rank
alone is therefore insufficient.  A matrix may have full rank while all its
positive combinations cover only a cone.

For a planar rigid body the twist space has dimension three; for a spatial body
it has dimension six.  Generic frictionless point contacts require at least four
contacts for planar first-order form closure and at least seven for spatial
first-order form closure.  Special geometries and higher-order effects require
separate analysis.

== Friction Cone

Coulomb friction:

$ norm(f_t) <= mu f_n, quad f_n >= 0 $

This is the isotropic Coulomb friction cone.  When sticking, the force lies
inside the cone.  During sliding, $norm(f_t)=mu f_n$ and $f_t$ opposes the
tangential slip direction.  The coefficient $mu$ and contact normal are model
parameters with uncertainty; robust grasps retain margin from the cone boundary.

The circular cone is often approximated by a polyhedral friction pyramid:

$ f_i=sum_j alpha_(i j) d_(i j), quad alpha_(i j)>=0, $

where $d_(i j)$ are primitive edge forces.  More edges reduce approximation
error but enlarge the optimization problem.  An inscribed pyramid is
conservative; a circumscribed one can falsely certify feasibility.

Other contact models change the wrench coordinates: a soft-finger contact can
transmit limited torsional moment about the normal, while a frictionless point
transmits only a compressive normal force.

== Grasp Matrix

For contact $i$ at object-frame position $p_i$, a point force $f_i$ produces
object wrench

$ cal(F)_i=mat(p_i times f_i;f_i) $

under the moment-force ordering.  Stack all contact coordinates in $f_c$; the
grasp matrix maps them to net object wrench:

$ cal(F)_("object") = G f_c $

Each block of $G$ incorporates the contact model and the transform from contact
coordinates to the object frame.  Internal forces satisfy

$ f_("int") in null(G). $

They change contact loads without changing the net object wrench.  Internal
preload can keep unilateral contacts closed and increase friction capacity, but
excess preload wastes actuator effort or damages the object.

The hand Jacobian maps joint velocity to contact-point velocity.  Virtual power
links the same contact force vector to joint torque:

$ tau=J_h^T f_c, quad cal(F)_("object")=G f_c. $

Thus a feasible grasp must satisfy both object equilibrium and hand actuator
limits.

== Force Closure

Force closure means feasible frictional contact forces can generate a balancing
wrench for any external object wrench.  If primitive contact wrenches are
$w_1,dots,w_m$, the positive wrench cone is

$ cal(W)={sum_i alpha_i w_i | alpha_i>=0}. $

Force closure requires this cone to equal the full wrench space.  After a
normalization such as $sum_i alpha_i=1$, a common geometric test is that the
origin lies strictly inside the convex hull of the primitive wrenches.

Informally:

$ span("contact wrench cone") = RR^6 $

with feasible positive contact-force magnitudes.

A linear-program test seeks strictly positive coefficients satisfying

$ sum_i alpha_i w_i=0, quad sum_i alpha_i=1, quad alpha_i>=epsilon. $

Why an interior convex-hull condition implies arbitrary wrench generation:
if $0$ is strictly interior, there is a ball $B_delta(0)$ inside the normalized
convex hull.  For any desired wrench $w!=0$,
$-delta w slash norm(w)$ belongs to that hull, so for some convex coefficients
$beta_i$,

$ -delta w slash norm(w)=sum_i beta_i w_i. $

Multiplying by $norm(w)/delta$ expresses $-w$ as a nonnegative combination of
contact wrenches.  Thus every external wrench can be balanced.

Full wrench rank is necessary but not sufficient because it ignores positivity.
The result also depends on the chosen friction-cone approximation and ignores
finite normal-force or actuator limits unless they are included explicitly.

*Form closure* uses geometry with frictionless unilateral constraints.  *Force
closure* uses the available frictional wrench cones.  Form closure implies a
strong geometric cage; force closure may be achieved with fewer contacts but
depends on friction and maintained preload.

== Motion-Force Duality

Power pairing is $cal(F)^T cal(V)$.  A wrench reciprocal to a twist performs zero
instantaneous work on it.  If admissible twists form a cone $cal(V)$, contact
wrenches that do nonpositive work on every admissible motion form its polar
cone.  This is the precise duality behind the statement:

- directions blocked by contact can support reaction wrench;
- unconstrained motion directions cannot be resisted by an ideal constraint;
- unilateral motion cones correspond to unilateral wrench cones, not merely
  orthogonal linear subspaces.

For equality constraints $A cal(V)=0$, admissible motion is $null(A)$ and ideal
constraint wrench lies in $range(A^T)$.  Inequality contacts require cone
duality and complementarity.

== Manipulation

Manipulation planning combines discrete contact modes with continuous robot and
object motion.  Common actions include free motion, grasp acquisition, rigid
grasp transport, pushing, rolling, sliding, pivoting, and regrasps.

At a quasistatic equilibrium,

$ G f_c + cal(F)_("ext")=0 $

along with friction cones, unilateral normal forces, hand torque limits, and
active contact-mode constraints.  Dynamic manipulation additionally includes
object inertia, impact, and possible contact impulses.

A useful planning decomposition is:

1. choose a contact mode or grasp and its geometric parameters;
2. test kinematic feasibility for the object and robot;
3. solve contact-force equilibrium with friction and actuator bounds;
4. plan a collision-free transition while maintaining the assumed contacts;
5. add margins and feedback for pose, friction, and force uncertainty.

Complementarity expresses mode consistency: a separating contact has zero
normal force, while a positive normal force requires zero normal gap.  Similar
stick-slip conditions couple tangential velocity and friction force.  Fixing a
mode converts some of these relations to equalities and inequalities; searching
all mode sequences is combinatorial.

== Takeaway

Grasping is the dual study of allowable object motion and achievable contact
wrench.  Contact mode defines the kinematic cone; friction and the grasp matrix
define the wrench cone; stable manipulation must satisfy both together with
robot and actuator constraints.
