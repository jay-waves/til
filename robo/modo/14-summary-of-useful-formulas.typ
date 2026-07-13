#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Formula reference],
  source: [Modern Robotics],
  revised: [2026-07-13],
  tags: ("robotics", "modern-robotics", "modo"),
)

#let argmin = math.op("arg min", limits: true)
#let exp = math.op("exp")
#let log = math.op("log")
#let sin = math.op("sin")
#let cos = math.op("cos")
#let acos = math.op("acos")
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

= Appendix A: Summary of Useful Formulas

This appendix is a convention-aware reference.  Twists use angular-linear
ordering $cal(V)=(omega,v)$ and wrenches use moment-force ordering
$cal(F)=(m,f)$.  A transform $T_(a b)$ maps coordinates from frame ${b}$ to
frame ${a}$.

== Mechanism Mobility

$ "dof" = "body freedoms" - "independent configuration constraints" $

For $N$ links including ground, $J$ joints with freedoms $f_i$, and
$d=3$ planar or $d=6$ spatial body freedom,

$ "dof" = d(N-1-J)+sum_(i=1)^J f_i. $

This Gruebler-Kutzbach count assumes generic independent constraints.  The
local mobility is determined by the rank of the actual constraint Jacobian.
Pfaffian velocity constraints have the form

$ A(theta)dot(theta)=0, quad dot(theta) in null(A). $

== Rotation Matrices

$ R in "SO"(3) arrow.l.r R^T R=I, quad det(R)=1. $

$ R^(-1)=R^T, quad R_(a c)=R_(a b)R_(b c), quad p_a=R_(a b)p_b. $

For $omega=(omega_1,omega_2,omega_3)$,

$ [omega]=mat(0,-omega_3,omega_2;
              omega_3,0,-omega_1;
              -omega_2,omega_1,0), $

$ [omega]x=omega times x, quad [omega]^T=-[omega], quad
  R[omega]R^T=[R omega]. $

Space and body angular velocities satisfy

$ [omega_s]=dot(R)R^T, quad [omega_b]=R^T dot(R), quad
  omega_s=R omega_b. $

=== Rotation Exponential and Logarithm

For unit $hat(omega)$, Rodrigues' formula is

$ R=e^([hat(omega)]theta)
  =I+sin theta[hat(omega)]+(1-cos theta)[hat(omega)]^2. $

Given $R$, the principal rotation angle satisfies

$ theta=acos((tr(R)-1)/2), quad theta in [0,pi]. $

For $theta$ away from $0$ and $pi$,

$ [hat(omega)]=1/(2sin theta)(R-R^T). $

At $theta approx 0$, use $log(R) approx (R-R^T)/2$.  At $theta approx pi$,
recover the axis from $R+I$ with a numerically stable branch; the axis sign is
ambiguous at exactly $pi$.

== Rigid Transforms

$ T = mat(R, p; 0, 1) $

$ T^(-1) = mat(R^T, -R^T p; 0, 1) $

$ T_(a c) = T_(a b) T_(b c) $

For a point in homogeneous coordinates,

$ mat(p_a;1)=T_(a b)mat(p_b;1). $

Space and body twists of $T=T_(s b)$ are

$ [cal(V)_s]=dot(T)T^(-1), quad
  [cal(V)_b]=T^(-1)dot(T). $

== Screws and Twist Exponentials

A screw axis and its matrix representation are

$ cal(S)=mat(omega;v), quad
  [cal(S)]=mat([omega],v;0,0). $

For a revolute axis through $q$ in unit direction $hat(omega)$,

$ omega=hat(omega), quad v=-hat(omega) times q. $

For a helical axis with pitch $h$,
$v=-hat(omega)times q+h hat(omega)$.  For unit prismatic direction $v$,
$omega=0$.

The twist generated at joint rate $dot(theta)$ is
$cal(V)=cal(S)dot(theta)$.

For $norm(omega)=1$,

$ e^([cal(S)] theta)
  =mat(e^([omega]theta),
       (I theta+(1-cos theta)[omega]+(theta-sin theta)[omega]^2)v;
       0,1). $

For $omega=0$,

$ e^([cal(S)] theta) = mat(I, v theta; 0, 1) $

For $T=mat(R,p;0,1)$ with $R!=I$, compute
$[hat(omega)]theta=log(R)$ and

$ G(theta)=I theta+(1-cos theta)[hat(omega)]
  +(theta-sin theta)[hat(omega)]^2, $

$ v=G(theta)^(-1)p, quad
  log(T)=mat([hat(omega)]theta,v theta;0,0). $

For pure translation, $theta=norm(p)$ and $v=p slash norm(p)$ when $p!=0$.

== Adjoint and Duality

$ Ad_T = mat(R, 0; [p] R, R) $

Twist transform:

$ cal(V)_s = Ad_T cal(V)_b $

Composition and inverse identities:

$ Ad_(T_1T_2)=Ad_(T_1)Ad_(T_2), quad
  Ad_(T^(-1))=(Ad_T)^(-1). $

For $T_(a b)$,

$ cal(V)_a=Ad_(T_(a b))cal(V)_b, quad
  cal(F)_a=Ad_(T_(b a))^T cal(F)_b. $

Wrenches use the inverse-transpose relation because power is invariant:

$ cal(F)^T cal(V) = "constant across frames" $

The small adjoint for $cal(V)=(omega,v)$ is

$ ad_(cal(V))=mat([omega],0;[v],[omega]), $

and the twist Lie bracket is
$[cal(V)_1,cal(V)_2]=ad_(cal(V)_1)cal(V)_2$.

== POE and Jacobians

Space POE:

$ T(theta) = e^([cal(S)_1] theta_1) dots e^([cal(S)_n] theta_n) M $

Body POE:

$ T(theta) = M e^([cal(B)_1] theta_1) dots e^([cal(B)_n] theta_n) $

$ cal(S)_i=Ad_M cal(B)_i, quad
  cal(B)_i=Ad_(M^(-1))cal(S)_i. $

Velocity:

$ cal(V) = J(theta) dot(theta) $

Space Jacobian columns:

$ J_(s 1)=cal(S)_1, quad
  J_(s i)=Ad_(e^([cal(S)_1]theta_1)dots
  e^([cal(S)_(i-1)]theta_(i-1)))cal(S)_i. $

Body Jacobian columns:

$ J_(b n)=cal(B)_n, quad
  J_(b i)=Ad_(e^(-[cal(B)_n]theta_n)dots
  e^(-[cal(B)_(i+1)]theta_(i+1)))cal(B)_i. $

$ J_s=Ad_(T_(s b))J_b, quad J_b=Ad_(T_(b s))J_s. $

Statics:

$ tau = J(theta)^T cal(F) $

The twist and wrench must be expressed in dual coordinates matching the chosen
Jacobian.

== Pseudoinverse and Manipulability

For full row rank $J in RR^(m times n)$ with $n>=m$,

$ J^dagger=J^T(J J^T)^(-1), quad J J^dagger=I. $

For full column rank with $n<=m$,

$ J^dagger=(J^T J)^(-1)J^T, quad J^dagger J=I. $

General redundant solution:

$ dot(theta)=J^dagger cal(V)+(I-J^dagger J)eta. $

Damped least squares:

$ dot(theta)=J^T(J J^T+lambda^2 I)^(-1)cal(V). $

For $J=U Sigma V^T$, velocity-ellipsoid axes are columns of $U$ with
semiaxis lengths $sigma_i$.  When $J$ has full row rank,

$ cal(V)^T(J J^T)^(-1)cal(V)=1, quad
  cal(F)^T(J J^T)cal(F)=1. $

Force semiaxes are $1/sigma_i$.  Use SVD directly at rank deficiency.

== Dynamics and Control

$ L=K-P, quad
  tau=d/(d t)(partial L slash partial dot(theta))
  -partial L slash partial theta. $

$ tau=M(theta)ddot(theta)+c(theta,dot(theta))+g(theta)
  +J(theta)^T cal(F)_("tip"). $

$ K=1/2 dot(theta)^T M(theta)dot(theta), quad
  M=M^T>0. $

$ ddot(theta)=M(theta)^(-1)
  (tau-c-g-J^T cal(F)_("tip")). $

In computation, solve the linear system instead of explicitly forming
$M^(-1)$.

Rigid-body spatial dynamics:

$ cal(F)=cal(G)dot(cal(V))-ad_(cal(V))^T cal(G)cal(V), quad
  K=1/2 cal(V)^T cal(G)cal(V). $

Operational-space inertia, when $J$ has full row rank:

$ Lambda=(J M^(-1)J^T)^(-1). $

Computed torque:

$ tau = M(theta) (ddot(theta)_d + K_d dot(e) + K_p e) + c(theta, dot(theta)) + g(theta) $

Resolved-rate motion control:

$ dot(theta)=J^dagger(cal(V)_d+K_p cal(V)_("err")). $

Impedance target:

$ M_d ddot(e)+B_d dot(e)+K_d e=cal(F)_("ext"). $

== Trajectory Formulas

$ theta(t)=theta(s(t)), $

$ dot(theta)=theta'(s)dot(s), quad
  ddot(theta)=theta'(s)ddot(s)+theta''(s)dot(s)^2. $

Joint-space straight path:

$ theta(s)=theta_0+s(theta_1-theta_0). $

Rest-to-rest cubic and rest-to-rest/zero-acceleration quintic, with
$r=t/T$:

$ s_3(r)=3r^2-2r^3, $

$ s_5(r)=10r^3-15r^4+6r^5. $

Screw interpolation:

$ T(s)=T_0 exp(log(T_0^(-1)T_1)s). $

Straight translation with geodesic rotation:

$ p(s)=p_0+s(p_1-p_0), quad
  R(s)=R_0 exp(log(R_0^T R_1)s). $

== Contact and Grasping

Coulomb cone:

$ norm(f_t)<=mu f_n, quad f_n>=0. $

Grasp and hand mappings:

$ cal(F)_("object")=G f_c, quad tau=J_h^T f_c. $

Internal forces lie in $null(G)$.  Force closure requires the positive cone of
feasible primitive contact wrenches to span the full object wrench space; full
matrix rank alone is not sufficient.

== Mobile-Robot Kinematics

Differential drive with wheel radius $r$ and half-track $l$:

$ v=r/2(dot(phi)_R+dot(phi)_L), quad
  omega=r/(2l)(dot(phi)_R-dot(phi)_L). $

$ dot(x)=v cos theta, quad dot(y)=v sin theta, quad dot(theta)=omega. $

Kinematic car with wheelbase $L$ and steering $phi$:

$ dot(x)=v cos theta, quad dot(y)=v sin theta, quad
  dot(theta)=v/L tan phi. $

== Takeaway

Most robotics errors are convention errors.  Verify dimensions, frame labels,
ordering, signs, rank assumptions, and singular limiting cases before trusting a
formula or numerical result.
