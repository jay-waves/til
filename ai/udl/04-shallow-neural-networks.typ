#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Book chapter 3],
  source: [Understanding Deep Learning],
  revised: [2026-07-13],
  tags: ("deep-learning", "udl"),
)

#let argmin = math.op("arg min", limits: true)
#let argmax = math.op("arg max", limits: true)
#let ReLU = math.op("ReLU")
#let softmax = math.op("softmax")
#let sigmoid = math.op("sigmoid")
#let KL = math.op("KL")
#let E = math.op("E")
#let Pr = math.op("Pr")
#let Var = math.op("Var")
#let Cov = math.op("Cov")
#let ELBO = math.op("ELBO")
#let diag = math.op("diag")
#let sign = math.op("sign")

= PDF 04: Shallow Neural Networks

== Motivation

Linear regression can only describe a straight-line relationship. Shallow neural networks are only slightly more complicated, but they can represent continuous piecewise-linear functions and, with enough hidden units, approximate arbitrarily complex continuous relationships.

The chapter begins with a scalar-input, scalar-output example with three hidden units and ten parameters:

$ y = f(x, phi) = phi_0 + phi_1 a(theta_(1 0) + theta_(1 1) x) + phi_2 a(theta_(2 0) + theta_(2 1) x) + phi_3 a(theta_(3 0) + theta_(3 1) x) $

The computation has three stages:

1. compute linear functions of the input
2. apply a nonlinear activation to each result
3. linearly combine the activations and add an output offset

The activation used throughout the chapter is the rectified linear unit:

$ a(z) = ReLU(z) = cases(0 & z < 0, z & z >= 0) $

It clips negative values to zero and leaves positive values unchanged.

== Hidden Units and Output Combination

The three hidden units in the introductory example are:

$ h_1 = a(theta_(1 0) + theta_(1 1) x) $

$ h_2 = a(theta_(2 0) + theta_(2 1) x) $

$ h_3 = a(theta_(3 0) + theta_(3 1) x) $

The output is then:

$ y = phi_0 + phi_1 h_1 + phi_2 h_2 + phi_3 h_3 $

Each hidden unit contains a line clipped by a ReLU. The point where the pre-activation crosses zero becomes a joint in the final function:

$ x = - theta_(d 0) / theta_(d 1) $

When a hidden unit is clipped, it is inactive; otherwise it is active. Each linear region of the output corresponds to one activation pattern. If the active set in a region is $cal(A)$, then:

$ y = phi_0 + sum_(d in cal(A)) phi_d (theta_(d 0) + theta_(d 1) x) $

and the slope in that region is:

$ (d y) / (d x) = sum_(d in cal(A)) phi_d theta_(d 1) $

With three hidden units there can be up to four linear regions. More generally, a scalar-input ReLU network with $D$ hidden units has at most $D$ joints and $D + 1$ linear regions.

== Universal Approximation

For a scalar input and $D$ hidden units:

$ h_d = ReLU(theta_(d 0) + sum_(i=1)^(D_i) theta_(d i) x_i) $

$ y = phi_0 + sum_(d=1)^D phi_d h_d $

Adding hidden units adds potential joints, so the piecewise-linear approximation can follow a continuous target function more closely. The universal approximation theorem states that there exists a finite-width one-hidden-layer network that can approximate any continuous function on a compact domain to arbitrary precision.

This is an existence result. It does not guarantee that the required number of hidden units is small, that gradient-based training will find the parameters, or that the learned model will generalize from finite data.

== Multivariate Outputs

For multiple outputs, the network can share hidden units and use a different linear combination for each output. With scalar input, four hidden units, and two outputs:

$ y_1 = phi_(1 0) + phi_(1 1) h_1 + phi_(1 2) h_2 + phi_(1 3) h_3 + phi_(1 4) h_4 $

$ y_2 = phi_(2 0) + phi_(2 1) h_1 + phi_(2 2) h_2 + phi_(2 3) h_3 + phi_(2 4) h_4 $

Because both outputs use the same hidden units, their joints occur at the same input locations. Their slopes and vertical offsets can differ because the output weights and biases are different.

== Multivariate Inputs

For input $x = [x_1, x_2, dots, x_(D_i)]^T$, each hidden unit receives an affine function of all input coordinates:

$ h_d = a(theta_(d 0) + sum_(i=1)^(D_i) theta_(d i) x_i) $

For a two-dimensional input, the pre-activation is a plane. The ReLU clips the part of the plane below zero, and the zero contour is a line in input space. In higher dimensions, the zero contour is a hyperplane.

The network partitions input space into convex regions:

$ theta_(d 0) + sum_(i=1)^(D_i) theta_(d i) x_i = 0 $

Within each region, the activation pattern is fixed and the network is affine. For multiple outputs, the regions are shared, but each output can contain a different affine function within those regions.

The number of regions can grow extremely quickly with input dimension. With $D_i$ hyperplanes aligned to the $D_i$ coordinate axes, the input space is divided into $2^(D_i)$ orthants. With more hidden units than input dimensions, shallow networks can usually create far more regions.

For $D_i >= 2$ and $D$ hidden units, the maximum number of regions is:

$ sum_(j=0)^(D_i) binom(D, j) $

== General Shallow Network

A shallow network maps $x in RR^(D_i)$ to $y in RR^(D_o)$ through one hidden layer $h in RR^D$:

$ h_d = a(theta_(d 0) + sum_(i=1)^(D_i) theta_(d i) x_i) $

$ y_j = phi_(j 0) + sum_(d=1)^D phi_(j d) h_d $

The parameters are the input-to-hidden weights and biases $theta$ and the hidden-to-output weights and biases $phi$.

The activation must be nonlinear. If there is no activation function, or if the activation is linear, the whole input-output mapping collapses to one affine function. The ReLU is emphasized because it makes the geometry easy to characterize and has useful training behavior on its positive side.

== Terminology

- pre-activation: value before the nonlinearity
- activation: value after the nonlinearity
- weight: slope parameter
- bias: offset parameter
- MLP: network with at least one hidden layer
- fully connected: all units in adjacent layers are connected

The input layer holds the input variables, the hidden layer holds the hidden units or neurons, and the output layer holds the predictions. A feed-forward network has acyclic connections flowing from input to output. A fully connected network connects every element in one layer to every element in the next.

The chapter notes that the biological interpretation of neural networks is weak. The term comes from a superficial similarity between densely connected computational nodes and biological neurons, not from a faithful model of brain computation.

== Activation Function Notes

The ReLU derivative is one for positive inputs and zero for negative inputs. This helps training compared with saturated sigmoid or tanh units, whose derivatives can become close to zero for large positive or negative inputs.

The drawback is the dying ReLU problem. If all examples drive a unit's pre-activation below zero, its output is zero and the gradient with respect to incoming weights is locally flat.

Several variants address this:

- leaky ReLU: small negative slope instead of zero
- parametric ReLU: learned negative slope
- concatenated ReLU: keeps both positive and negative clipped versions
- smooth alternatives such as SoftPlus, GELU, SiLU, ELU, SELU, Swish, and HardSwish

The book continues with basic ReLUs because their piecewise-linear geometry is simple and useful for analysis.

== Linear, Affine, and Nonlinear

Technically, a linear function obeys superposition: $f(a + b) = f(a) + f(b)$ and $f(c a) = c f(a)$. Adding a bias makes the function affine rather than strictly linear. Machine learning writing often calls both "linear"; the book follows that convention. ReLU and networks containing ReLUs are nonlinear.

== Takeaway

Shallow neural networks compute affine functions, clip them with nonlinear activations, and recombine the resulting hidden units. With ReLUs, this creates continuous piecewise-affine functions whose regions are determined by learned joints or hyperplanes.
