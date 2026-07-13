#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Configuration space and planning methods],
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

= Chapter 10: Motion Planning

Motion planning computes a feasible motion from a start configuration to a goal
while respecting obstacles and possibly differential or dynamic constraints.
It is distinct from trajectory generation: planning decides *where* to move;
trajectory generation often decides *when* to traverse an already selected
path.  In practice the two may be solved together.

== Configuration Space

Planning searches for a path:

$ q: [0, 1] -> cal(C)_("free") $

with:

$ q(0) = q_("start"), quad q(1) = q_("goal") $

and:

$ cal(C)_("free") = cal(C) - cal(C)_("obs") $

Each point in configuration space $cal(C)$ represents the entire robot.  A
workspace collision therefore becomes a forbidden region
$cal(C)_("obs")$.  Workspace planning is insufficient for manipulators because
different joint configurations can realize the same end-effector pose, and
self-collision and joint limits are configuration-space constraints.

The topology matters.  An unrestricted revolute joint lies on a circle, so
angles near $-pi$ and $pi$ are close; a free rigid body lies on $"SE"(3)$, not
ordinary $RR^6$.  Distance metrics, interpolation, and sampling must respect
this topology and the relative scale of heterogeneous coordinates.

=== Configuration Obstacles

For a translating rigid body, configuration obstacles can be constructed by
expanding workspace obstacles by the reflected robot shape, a Minkowski-sum
operation.  For articulated robots, explicit construction is usually
intractable; planners query collision at sampled configurations instead.

$cal(C)_("free")$ can contain disconnected components, narrow passages, and
lower-dimensional corridors.  Valid start and goal configurations do not imply
that a connecting path exists.

== Planner Properties

Planner guarantees must be stated precisely:

- *Complete:* finds a path when one exists and otherwise terminates with proof
  of failure.
- *Resolution complete:* complete relative to a chosen discretization or feature
  resolution; a finer grid may change the answer.
- *Probabilistically complete:* if a robust feasible path exists, the
  probability of finding one approaches one as samples grow.
- *Optimal:* returns a minimum-cost path; *asymptotically optimal* converges in
  cost with increasing computation.

Most practical high-dimensional planners are probabilistically complete, not
complete.  A timeout is evidence only that a path was not found within the
budget, not that no path exists.

== Graph Search

A graph contains vertices $V$ and edges $E$, with nonnegative edge cost.  A path
search typically maintains a frontier and cost-to-come.  Dijkstra's algorithm
expands the vertex with smallest $g$ and is optimal for nonnegative costs.
A-star prioritizes

$ f(n) = g(n) + h(n) $

where $h$ estimates cost-to-go.  If $h$ never overestimates the true remaining
cost, it is admissible; if
$h(n)<=c(n,n')+h(n')$, it is consistent.  With the usual graph-search rules, a
consistent heuristic preserves optimality and avoids reopening closed nodes.

The optimality argument is a lower-bound argument.  If $h(n)<=h^*(n)$, then

$ f(n)=g(n)+h(n)<=g(n)+h^*(n), $

so $f(n)$ is a lower bound on the cost of any solution continuing through $n$.
When A-star removes a goal with cost $C$ and every frontier node has $f>=C$, no
unexplored continuation can have cost below $C$.

Consistency implies admissibility when $h("goal")=0$: applying
$h(n)<=c(n,n')+h(n')$ repeatedly along an optimal path gives
$h(n)<=h^*(n)$.  It also makes $f$ nondecreasing along a path because

$ g(n')+h(n')>=g(n)+c(n,n')+h(n')>=g(n)+h(n). $

Parent pointers reconstruct the path after the goal is reached.  Search
correctness also depends on the graph: an optimal graph path may be a poor or
invalid approximation of the continuous problem if edges are too coarse or
collision checks are inadequate.

== Grid Methods

Grid planners discretize $cal(C)$ and connect neighboring cells or samples.
They are simple and resolution complete under appropriate assumptions, but the
number of cells grows exponentially with dimension.  Connectivity choices
affect metric bias and corner cutting; diagonal edges require their own
collision checks.

Multi-resolution grids refine only where needed.  An adaptive method must
preserve connectivity across different cell sizes and refine ambiguous boundary
cells enough to support its claimed resolution guarantee.

For systems with motion constraints, graph edges cannot be arbitrary straight
segments.  A *state lattice* uses short dynamically feasible motion primitives,
often generated by integrating controls.  Search then occurs over state, not
configuration alone, when velocity affects future feasibility.

== Sampling Methods

Sampling planners avoid representing all of $cal(C)_("free")$.  Their behavior
depends on a sampler, metric, nearest-neighbor structure, local planner, and
collision checker.

=== Rapidly Exploring Random Tree

RRT grows a tree rooted at the start:

1. sample $q_("rand")$
2. find nearest $q_("near")$
3. steer from $q_("near")$ toward the sample, usually by a bounded step
4. add the edge if collision-free

Its Voronoi bias tends to expand into unexplored regions.  Goal bias samples the
goal occasionally; bidirectional RRT-Connect grows trees from both endpoints
and is often effective for geometric manipulation planning.  Basic RRT is
feasible-path oriented and produces jagged, nonoptimal paths.  RRT-star adds
near-neighbor parent selection and rewiring to achieve asymptotic optimality
under its assumptions.

=== Probabilistic Roadmap

PRM samples many collision-free configurations, connects nearby pairs with a
local planner, and searches the resulting roadmap.  It is a *multi-query*
method: expensive roadmap construction can be reused for many start-goal
requests in a static environment.  Lazy PRM postpones edge collision checks
until a candidate graph path needs them.

RRT is naturally single-query and explores from the current problem's start;
PRM amortizes work across queries.  Both struggle with narrow passages because
their volume is small, and both require a local connection that matches the
system's kinematics or dynamics.

The basis of probabilistic completeness is measure.  Suppose a robust feasible
path can be covered by finitely many sampling neighborhoods, each with sampling
probability at least $p>0$.  The probability that one specified neighborhood is
still empty after $N$ independent samples is

$ (1-p)^N arrow.r 0 " as " N arrow.r infinity. $

A union bound over finitely many neighborhoods also tends to zero.  Positive
clearance supplies the nonzero-volume neighborhoods; a zero-clearance passage
does not satisfy this argument.

#note[Nearest-neighbor distance is part of the planner model.  Raw Euclidean
distance on mixed meters and radians can distort exploration; use normalized or
task-informed weights and respect angle wraparound.]

== Potential Fields

$ U(q) = U_("att")(q) + U_("rep")(q) $

$ dot(q) = - nabla U(q) $

A typical attractive potential is quadratic near the goal, while a repulsive
potential grows as obstacle distance approaches a safety threshold.  Workspace
distance gradients can be mapped to joints through point Jacobians.

For example,

$ U_("att")(q)=1/2 zeta norm(q-q_g)^2, quad
  nabla U_("att")=zeta(q-q_g). $

With obstacle distance $d(q)$ and influence distance $d_0$, define

$ U_("rep")(q)=cases(
  1/2 eta(1/d(q)-1/d_0)^2 & d(q)<=d_0,
  0 & d(q)>d_0). $

Inside the influence region, the chain rule gives

$ nabla U_("rep")
  =-eta(1/d-1/d_0)1/d^2 nabla d(q). $

If $d$ is measured between a robot point $x(q)$ and an obstacle with outward
unit normal $n$, then $nabla d(q)=J_x(q)^T n$.

Potential fields are useful local planners, reactive controllers, sampling
biases, or search heuristics, but plain gradient descent can stop at local
minima, oscillate in corridors, or fail to pass between close obstacles.
Artificial potentials also need careful scaling across robot points and joints.

A navigation function is designed to have a unique minimum at the goal on a
restricted class of spaces, but constructing one for a general articulated
robot is difficult.  Random escapes or a global planner can complement a local
potential method without turning it into a complete planner.

== Optimization and Smoothing

Trajectory optimization parameterizes a path or trajectory and minimizes a
cost such as length, smoothness, time, control effort, or obstacle proximity:

$ min_(q_0,dots,q_N) sum_k l(q_k,q_(k+1)) $

subject to endpoint, collision, joint, kinematic, and possibly dynamic
constraints.  Gradients from signed distance and robot Jacobians make local
optimization efficient, but the result depends on initialization and can settle
in the wrong homotopy class.  A sampling-based path often provides a feasible
initial guess.

Smoothing removes unnecessary detours after a feasible path is found.  Random
shortcutting repeatedly replaces a path segment by a shorter valid local
connection.  Spline fitting improves derivatives but can move the path into
collision or violate limits, so the modified path must be checked again.

== Planning Problem Variants

- *Point-to-point:* start and goal configurations are fixed.
- *Goal-set planning:* any configuration satisfying a task condition is valid;
  sampling IK solutions jointly with planning avoids committing too early to a
  bad goal branch.
- *Multiple robots or movable objects:* the composite configuration dimension
  grows and introduces coordination constraints.
- *Kinodynamic planning:* acceleration, velocity, nonholonomic, or actuator
  limits require search over state and controls.
- *Online planning:* changes in the environment require sensing, replanning,
  and a safe behavior while no new path is available.

== Takeaway

Motion planning is continuous feasibility represented through collision queries,
graphs, sampling, or optimization in configuration or state space.  Planner
guarantees are only as meaningful as the topology, local connections, and
collision checks used to construct that representation.
