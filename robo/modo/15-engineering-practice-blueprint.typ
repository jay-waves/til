#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Numerical robotics, software architecture, and validation],
  source: [Modern Robotics and engineering synthesis],
  revised: [2026-07-13],
  tags: ("robotics", "engineering", "numerical-methods", "modo"),
)

#let exp = math.op("exp")
#let atan2 = math.op("atan2")
#let rank = math.op("rank")
#let det = math.op("det")
#let tr = math.op("tr")
#let Ad = math.op("Ad")
#let J = math.op("J")
#let diag = math.op("diag")
#let null = math.op("null")
#let argmin = math.op("arg min", limits: true)

#let ddot(x) = math.dot.double(x)

= Robotics Engineering Practice Blueprint

This note turns the chapter-level models into an executable robotics system.
Its scope is numerical computation, data contracts, software boundaries,
verification, calibration, real-time execution, and safety.  The companion
chapter notes retain the mathematical theory and derivations.

== System Decomposition

A maintainable stack separates responsibilities:

1. *Geometry and model data:* frames, transforms, joint topology, inertial and
   collision properties, limits, actuator and sensor metadata.
2. *Numerical kernel:* robust $"SO"(3)$/$"SE"(3)$ operations, POE, Jacobians,
   dynamics, integration, linear algebra, and automatic differentiation.
3. *Estimation:* timestamped sensor ingestion, calibration, filtering, state and
   uncertainty estimation.
4. *Solvers:* IK, constrained least squares, contact equilibrium, trajectory
   optimization, graph search, and sampling planners.
5. *Control:* reference shaping, whole-body optimization, torque/velocity
   control, contact modes, saturation, and safety supervision.
6. *Runtime:* scheduling, communication, logging, replay, health monitoring,
   configuration management, and fault handling.

The layers exchange typed values rather than bare arrays.  A pose should carry
parent and child frame identities; a twist or wrench should carry expression
frame, reference point, ordering, units, and timestamp.

== Coordinate and Unit Contract

Adopt one project-wide convention and enforce it at API boundaries:

- $T_(a b)$ maps coordinates from ${b}$ to ${a}$.
- twists use $(omega,v)$; wrenches use $(m,f)$;
- spatial quantities are expressed in the fixed frame, body quantities in the
  moving frame;
- revolute coordinates use radians and prismatic coordinates use meters;
- inertia uses $"kg" dot "m"^2$, torque $"N" dot "m"$, and force $"N"$;
- timestamps use a monotonic clock and declare whether samples are measured,
  estimated, commanded, or predicted.

At runtime, assert

$ R^T R approx I, quad det(R) approx 1, quad M=M^T, quad
  lambda_("min")(M)>0. $

Do not silently normalize invalid rotations or inertias in production.  Report
the correction magnitude and reject data beyond a declared tolerance.

== Robot Model Pipeline

=== Source Model

URDF supplies the link tree, joint axes and origins, limits, visual/collision
geometry, and link inertias.  It does not fully specify transmission friction,
motor electrical limits, backlash, cable routing, flexible modes, controller
bandwidth, or general closed loops.  Keep those in explicit companion schemas.

Model ingestion should:

1. resolve mesh and package paths deterministically;
2. validate masses, symmetric inertias, joint axes, limits, and tree topology;
3. transform inertias from declared inertial frames to computation frames;
4. build both kinematic and collision models from the same joint ordering;
5. generate immutable indices and frame identifiers;
6. store a hash of all source artifacts with logs and test results.

For a center-of-mass inertia $I_C$, mass $m$, and offset $q$ to a parallel frame,

$ I_q=I_C+m((q^T q)I-q q^T). $

The spatial inertia must be transformed using the adjoint congruence appropriate
to the chosen twist convention.  A sign error here can leave static kinematics
correct while corrupting every dynamic calculation.

=== Generated Representations

From one validated model, generate:

- home pose $M$ and space/body screw lists;
- parent-child transforms and link spatial inertias;
- collision pairs, excluding adjacent pairs only when physically justified;
- actuator-coordinate maps and reflected rotor inertias;
- serialization metadata for logs and network messages.

Cross-check generated POE forward kinematics against tree-transform forward
kinematics over randomized configurations.

== Lie-Group Numerical Kernel

=== Rotation Projection

After noisy estimation or long integration, a matrix may drift from $"SO"(3)$.
For $A=U Sigma V^T$, the closest proper rotation in Frobenius norm is

$ R=U diag(1,1,det(U V^T))V^T. $

Projection is a repair operation, not a substitute for a group-preserving
integrator.  Record when and how often it is needed.

=== Stable Exponential and Logarithm

For small $theta$, evaluate coefficients with series rather than subtracting
nearly equal numbers:

$ sin(theta)/theta &= 1-theta^2/6+theta^4/120+O(theta^6), \
  (1-cos(theta))/theta^2 &= 1/2-theta^2/24+theta^4/720+O(theta^6), \
  (theta-sin(theta))/theta^3 &= 1/6-theta^2/120+O(theta^4). $

For $log(R)$, clamp $(tr(R)-1)/2$ to $[-1,1]$ before inverse cosine.  Use a
small-angle antisymmetric approximation near zero and a diagonal-based axis
recovery near $pi$.  Preserve a continuous branch when processing a trajectory.

Integrate a body twist with

$ T_(k+1)=T_k exp([cal(V)_(b,k)]Delta t) $

or a space twist with left multiplication.  Mixing the multiplication side and
twist convention produces a state-dependent error.

=== Linear Algebra Policy

Never form an explicit matrix inverse merely to solve a system.  Use:

- Cholesky/LDLT for symmetric positive-definite systems;
- QR for full-rank least squares;
- SVD for rank-revealing solves and pseudoinverses;
- sparse factorizations for trajectory and constrained dynamics systems.

For $J=U Sigma V^T$, define a numerical rank through scale-aware threshold

$ sigma_i > epsilon_("abs")+epsilon_("rel")sigma_1. $

Log singular values and residuals, not just a Boolean success flag.

== Kinematic Verification

=== Forward Kinematics

Required invariants:

$ T(0)=M, quad T^(-1)T=I, quad
  T_(a b)T_(b c)=T_(a c). $

Test revolute axes with small positive motion and the right-hand rule; test
prismatic axes against their declared translation direction.  Use randomized
joint vectors including limit-adjacent and near-singular poses.

=== Jacobian by Finite Difference

For step $h$ and basis vector $e_i$, a central body-twist check is

$ cal(J)_(b,i)^("FD")
  =1/(2h)(log(T(theta)^(-1)T(theta+h e_i))^("vee")
   -log(T(theta)^(-1)T(theta-h e_i))^("vee")). $

Both increments are therefore expressed in the same current body frame.  Sweep
$h$: truncation error dominates when $h$ is large, floating-point cancellation
when it is too small.  Verify

$ J_s(theta)=Ad_(T(theta))J_b(theta). $

Automatic differentiation is useful for coordinate maps, but derivatives of
manifold values still require an explicit local-coordinate convention.

== Numerical Inverse Kinematics

Given body error

$ e(theta)=log(T(theta)^(-1)T_d)^("vee"), $

solve a weighted damped step

$ Delta theta=argmin_(z)
  norm(W_e^(1/2)(J_b z-e))^2
  +lambda^2 norm(W_q^(1/2)z)^2. $

The normal-equation form is

$ (J_b^T W_e J_b+lambda^2 W_q)Delta theta=J_b^T W_e e, $

but QR/SVD is preferable when conditioning matters.  Translation and rotation
weights encode a characteristic length and must not be chosen implicitly.

=== Globalization and Constraints

Use a trust region or backtracking line search:

$ theta_(k+1)=theta_k+alpha_k Delta theta, quad 0<alpha_k<=1, $

accepting a step only when the selected merit function decreases sufficiently.
For hard bounds solve a bounded least-squares or QP subproblem instead of
clipping an unconstrained result.

Termination states are distinct:

- success: angular and linear residuals below tolerances;
- stagnation: step small but residual unacceptable;
- infeasible/limit-bound: active constraints prevent improvement;
- ill-conditioned: effective rank below task requirement;
- budget exhausted: iteration or wall-clock limit reached.

Use multiple seeds for branch discovery and continuation from the previous
solution for trajectory tracking.  Deduplicate solutions modulo valid revolute
wraps and forward-check each result.

== Closed-Chain and Contact Solvers

For closure residual $g(theta)=0$, Newton or SQP uses

$ A(theta)Delta theta=-g(theta), quad A=partial g slash partial theta. $

Track assembly mode by continuation, bound the step, and monitor
$sigma_("min")(A)$.  Redundant entries of a transform residual should not be
treated as independent constraints.

Rigid contact with unilateral normal gap $phi_n$ and normal force $lambda_n$
uses complementarity

$ phi_n>=0, quad lambda_n>=0, quad phi_n lambda_n=0. $

Friction adds cone constraints and stick/slip modes.  Depending on fidelity and
runtime needs, use a fixed-mode QP, nonlinear complementarity solver, cone
program, or time-stepping contact method.  Always report residual, cone margin,
and active mode.

== Dynamics and Simulation

=== Cross-Checks

At representative configurations verify

$ M=M^T, quad lambda_("min")(M)>0, $

and inverse/forward consistency:

$ tau="ID"(theta,dot(theta),ddot(theta),cal(F)_("ext")) $

must recover $ddot(theta)$ through forward dynamics within tolerance.  Compare
mass-matrix columns obtained by unit-acceleration inverse-dynamics calls with the
direct mass-matrix routine.

With gravity, damping, actuation, and external work disabled, monitor

$ E=1/2 dot(theta)^T M(theta)dot(theta)+P(theta). $

Numerical energy drift should converge toward zero as the step is refined.

=== Integration

Explicit Euler is acceptable only for diagnostics at sufficiently small steps.
Semi-implicit Euler updates velocity before position and often behaves better for
mechanical systems.  Runge-Kutta methods improve local accuracy but do not
guarantee energy behavior.  Stiff contact, high gearing, or flexibility may
require implicit integration.

Choose the time step from the fastest modeled dynamics and controller sample
rate, then perform a convergence study.  Collision/contact events require event
detection or a time-stepping formulation; stepping across an impact without an
impulse model corrupts momentum.

== Parameter Identification and Calibration

Calibrate in an observable order:

1. sensor biases, scales, timing, and extrinsics;
2. joint zero offsets and kinematic geometry;
3. wheel radii/track width or other base parameters;
4. masses, centers of mass, and inertial combinations;
5. friction, motor constants, transmission elasticity, and delay.

For a model linear in dynamic parameters,

$ tau=Y(theta,dot(theta),ddot(theta))pi, $

estimate base parameters by weighted regularized least squares

$ hat(pi)=argmin_pi norm(W^(1/2)(Y pi-tau))^2
  +lambda^2 norm(L(pi-pi_0))^2. $

Design excitation trajectories so $Y$ is well conditioned while respecting
joint, velocity, acceleration, torque, collision, and temperature limits.
Validate on held-out trajectories; training residual alone does not establish
predictive accuracy.

== Trajectory Generation

Represent every trajectory with evaluators for position, velocity, and
acceleration and declared validity interval.  At segment boundaries test the
required $C^k$ continuity explicitly.

For a time-scaled path,

$ dot(theta)=theta'(s)dot(s), quad
  ddot(theta)=theta'(s)ddot(s)+theta''(s)dot(s)^2. $

Evaluate extrema between samples: polynomial derivatives can peak away from
knots.  Check joint position, velocity, acceleration, jerk, torque, power,
collision, and contact constraints over the full interval.  Task-space paths
also require continuous IK branch tracking.

For time-optimal path parameterization, discretize or integrate the feasible
acceleration envelope

$ L(s,dot(s))<=ddot(s)<=U(s,dot(s)), $

with explicit handling of zero-inertia switching points, velocity caps, and
model margins.  Reconstruct time using $d t=d s slash dot(s)$ and treat stops
without division by zero.

== Collision and Motion Planning

=== Collision Service

Expose configuration validity, continuous edge validity, signed distance, and
nearest-point/Jacobian queries.  Use broad-phase bounding-volume pruning and a
geometry-accurate narrow phase.  Maintain separate visual and conservative
collision geometry.

Fixed-step edge checking needs a resolution derived from a bound on swept robot
motion; endpoint-only checks are invalid.  Cache only under immutable scene and
model versions.

=== Planner Selection

- low-dimensional, well-bounded spaces: grid/A-star;
- repeated queries in a static scene: PRM or a reusable roadmap;
- single high-dimensional geometric query: bidirectional RRT variants;
- dynamics/nonholonomic constraints: state lattice, kinodynamic RRT, or direct
  trajectory optimization;
- high-quality path with a feasible seed: local trajectory optimization.

Normalize distance across radians and meters, respect topology, and supply a
local steering method consistent with the model.  A timeout means “not found,”
not “infeasible.”

Postprocessing may shortcut or smooth a path, but every modified edge must be
collision-checked and all dynamic constraints revalidated after time scaling.

== Control and Real-Time Execution

=== Controller Contract

Each controller declares:

- required state, frames, rates, and uncertainty assumptions;
- output type: position, velocity, torque, current, or wrench;
- nominal period, deadline, and behavior on missed data;
- saturation, anti-windup, rate limiting, and fallback behavior;
- valid contact mode and transition conditions.

Do not allocate memory, load meshes, factor changing-size systems, or perform
unbounded logging in a hard real-time loop.  Preallocate workspaces and bound
solver iterations.

=== Constrained Whole-Body Control

A common QP chooses generalized acceleration and contact wrench:

$ min_(ddot(q),lambda) &
  norm(J ddot(q)+dot(J)dot(q)-cal(A)_d)^2_W
  +norm(ddot(q))^2_R, \
  "subject to" &
  M ddot(q)+h=S^T tau+J_c^T lambda, \
  &J_c ddot(q)+dot(J)_c dot(q)=0, \
  &tau_("min")<=tau<=tau_("max"), \
  &lambda in cal(K)_("friction"), $

plus joint, velocity, acceleration, and collision-damper constraints.  Use
slack variables with explicit priorities for tasks that may conflict, and log
which constraints or slacks are active.

=== Contact Transitions

Approach with bounded normal velocity, detect contact using filtered but timely
signals, ramp force/impedance references, reset or freeze incompatible
integrators, and define loss-of-contact behavior.  Test transitions against
sensor delay and false positives, not only steady contact.

== Mobile-Robot Estimation

For differential-drive increments,

$ Delta s=r/2(Delta phi_R+Delta phi_L), quad
  Delta theta=r/(2l)(Delta phi_R-Delta phi_L). $

Integrate the constant-curvature $"SE"(2)$ increment, using its straight limit
near $Delta theta=0$.  Propagate covariance through motion Jacobians and inflate
noise under detected slip.  Fuse encoders with IMU and exteroceptive updates to
bound drift.

Calibrate wheel radii and track width with both straight and turning datasets.
Validate forward/reverse symmetry and surfaces with different traction.

== Testing Strategy

=== Unit and Property Tests

- group identities for exp/log, inverse, composition, adjoint, and dual power;
- randomized POE versus tree forward kinematics;
- analytic Jacobians versus finite differences or automatic differentiation;
- symmetric positive-definite inertia and energy identities;
- inverse/forward dynamics round trips;
- trajectory boundary conditions and derivative continuity;
- collision edge cases, including touching, containment, and thin obstacles.

Use condition-aware tolerances

$ norm(r)<=epsilon_("abs")+epsilon_("rel")norm(x), $

not one global decimal threshold.

=== Scenario Tests

Build deterministic scenarios for nominal tracking, near singularity, joint
limits, unreachable goals, payload mismatch, sensor dropout, actuator saturation,
contact gain changes, wheel slip, planner timeout, and emergency stop.  Store
seeds and artifacts so every failure is replayable.

=== Simulation and Hardware Stages

1. pure numerical unit tests;
2. deterministic rigid-body simulation;
3. sensor/actuator noise, delay, saturation, and model mismatch;
4. software-in-the-loop with production messages and clocks;
5. hardware-in-the-loop or motor-disabled I/O tests;
6. low-energy hardware tests with reduced workspace and limits;
7. staged expansion to the operational envelope.

== Observability and Reproducibility

Log synchronized measured, estimated, desired, and commanded states; solver
status and residuals; active constraints; model/configuration hashes; controller
mode; safety events; and monotonic timestamps.  Logs must support deterministic
offline replay through estimators, solvers, and controllers.

Every experiment record should identify source revision, model artifact, numeric
precision, solver/version/options, calibration set, random seed, and hardware
configuration.  Plotting scripts are derived artifacts and should consume the
same typed logs as regression tests.

== Safety Architecture

Safety is an independent supervisory layer, not a final clamp inside the motion
controller.  Enforce layered limits:

- model and command validation before enabling;
- position, velocity, acceleration, torque/current, power, and temperature
  bounds;
- workspace, self-collision, environment, and contact-force limits;
- watchdogs for stale state, missed deadlines, solver failure, and communication
  loss;
- energy or power limiting for human contact where applicable;
- hardware emergency stop and safe torque-off independent of application code.

Define the safe state for each failure: controlled stop, gravity-support mode,
brake engagement, contact release, or power removal.  A generic “stop sending
commands” is unsafe when the actuator retains its last command.

== Delivery Checklist

Before deploying a new model, planner, or controller:

1. all coordinate/unit contracts are machine-checked;
2. unit, property, and scenario tests pass with stored tolerances;
3. model parameters are validated on held-out data;
4. final trajectories pass continuous collision and dynamic-limit checks;
5. worst-case solver time fits the deadline or has a tested fallback;
6. saturation, contact transitions, stale sensing, and emergency stop are tested;
7. logs contain enough state to reconstruct every decision;
8. model, calibration, software, and configuration versions are immutable and
   recoverable.

== Takeaway

A robotics implementation is credible only when its mathematical conventions,
numerical conditioning, model provenance, solver status, real-time behavior, and
safety response are explicit and testable.  The blueprint connects the chapter
equations to that evidence chain.
