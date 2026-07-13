#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Book chapter 7],
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

= PDF 08: Gradients and Initialization

== Training Requirements

Optimization algorithms such as SGD and Adam need gradients of the loss with respect to every weight and bias. Large neural networks can have enormous parameter counts, so these gradients must be computed efficiently at every iteration.

This chapter addresses two neural-network-specific problems:

- compute gradients efficiently with backpropagation
- initialize weights so activations and gradients remain numerically stable

For a network with three hidden layers:

$ h_1 = a(beta_0 + Omega_0 x) $

$ h_2 = a(beta_1 + Omega_1 h_1) $

$ h_3 = a(beta_2 + Omega_2 h_2) $

$ f(x, phi) = beta_3 + Omega_3 h_3 $

the parameters are all $beta_k$ and $Omega_k$. For each batch example, the optimizer needs derivatives of the individual loss with respect to every $beta_k$ and $Omega_k$.

== Backpropagation Intuition

Backpropagation is reverse-mode automatic differentiation. It repeatedly applies the chain rule:

$ (d z) / (d x) = (d f) / (d g) (d g) / (d x) $

The algorithm has two passes:

1. Forward pass: compute and store intermediate pre-activations, activations, model output, and loss.
2. Backward pass: start from the derivative of the loss and move backward through the graph, reusing derivatives already computed.

Two observations explain why this works:

- A weight's effect is scaled by its source activation, so activations from the forward pass must be stored.
- A change to an early parameter ripples through every later layer, so derivatives of later layers can be computed once and reused for all earlier parameters.

== Toy Scalar Example

The chapter illustrates backpropagation with a scalar composition:

$ f(x, phi) = beta_3 + omega_3 cos(beta_2 + omega_2 exp(beta_1 + omega_1 sin(beta_0 + omega_0 x))) $

with least-squares loss:

$ ell_i = (f(x_i, phi) - y_i)^2 $

Instead of deriving one huge expression for each parameter derivative, define intermediate values:

$ f_0 = beta_0 + omega_0 x_i $

$ h_1 = sin(f_0) $

$ f_1 = beta_1 + omega_1 h_1 $

$ h_2 = exp(f_1) $

$ f_2 = beta_2 + omega_2 h_2 $

$ h_3 = cos(f_2) $

$ f_3 = beta_3 + omega_3 h_3 $

$ ell_i = (f_3 - y_i)^2 $

The backward pass first computes derivatives with respect to intermediate variables in reverse order. Then parameter derivatives follow from:

$ (partial ell_i) / (partial beta_k) = (partial f_k) / (partial beta_k) (partial ell_i) / (partial f_k) $

$ (partial ell_i) / (partial omega_k) = (partial f_k) / (partial omega_k) (partial ell_i) / (partial f_k) $

Since $f_k = beta_k + omega_k h_k$, the derivative with respect to $omega_k$ is proportional to the stored source activation $h_k$.

== Backpropagation for ReLU Networks

For a sequential ReLU network, write the forward pass as:

$ f_0 = beta_0 + Omega_0 x_i $

$ h_k = a(f_(k - 1)) $

$ f_k = beta_k + Omega_k h_k $

where $a$ applies ReLU elementwise.

The ReLU derivative is:

$ (d ReLU(z)) / (d z) = cases(0 & z < 0, 1 & z > 0) $

At $z = 0$, implementations choose a convention. In practice this single point rarely matters for continuous inputs.

The backward pass alternates two operations:

- multiply by transposed weight matrices $Omega_k^T$
- pointwise mask by $I[f_(k - 1) > 0]$, the ReLU derivative

For the output-side layers:

$ (partial ell_i) / (partial beta_k) = (partial ell_i) / (partial f_k) $

$ (partial ell_i) / (partial Omega_k) = (partial ell_i) / (partial f_k) h_k^T $

For the first layer:

$ (partial ell_i) / (partial beta_0) = (partial ell_i) / (partial f_0) $

$ (partial ell_i) / (partial Omega_0) = (partial ell_i) / (partial f_0) x_i^T $

The per-example gradients are summed over the batch to form the SGD update. Backpropagation is compute-efficient because the major operations are matrix multiplications, but it is memory-intensive because the forward-pass intermediates must be kept for the backward pass.

== Algorithmic Differentiation

Modern frameworks such as PyTorch and TensorFlow perform algorithmic differentiation automatically. Each primitive operation knows its local derivative, and the framework records the computational graph needed for the backward pass.

These frameworks process batches in parallel, usually on GPUs. Batched data are represented as tensors: vectors are 1D tensors, matrices are 2D tensors, images with height, width, and channels are 3D tensors, and batched images are 4D tensors.

Backpropagation extends beyond purely sequential networks to arbitrary acyclic computational graphs. Branches can split and recombine; gradients accumulate along all downstream paths.

== Why Initialization Matters

Suppose weights are initialized from a zero-mean distribution with variance $sigma_Omega^2$ and biases are initialized to zero. If $sigma_Omega^2$ is too small, activations shrink layer by layer. If it is too large, activations grow layer by layer. Both cases can break finite-precision computation or make learning ineffective.

The same issue occurs in the backward pass. Multiplication by weight matrices can make gradients vanish or explode as they move back through many layers.

== Forward-Pass Variance

Consider adjacent layers:

$ h = a(f) $

$ f' = beta + Omega h $

Assume independent zero-mean weights and symmetric pre-activations. For ReLU, about half the inputs are clipped, so:

$ Var(f') approx 1 / 2 D_h sigma_Omega^2 Var(f) $

To keep variance stable through the forward pass:

$ sigma_Omega^2 = 2 / D_h $

where $D_h$ is the number of inputs to the weight matrix. This is He initialization for ReLU networks.

== Backward-Pass Variance

The backward pass multiplies by $Omega^T$. A similar variance argument gives:

$ sigma_Omega^2 = 2 / D_(h') $

where $D_(h')$ is the number of units in the layer the weights feed into.

If the matrix is not square, these forward and backward requirements differ. A compromise uses:

$ sigma_Omega^2 = 4 / (D_h + D_(h')) $

Appropriate initialization keeps both activations and gradients in a useful numeric range at the start of training.

== Example Training Code

The chapter includes PyTorch code that defines a small two-hidden-layer network, applies He initialization, uses mean-squared error, and trains with SGD plus momentum in minibatches. The main implementation lesson is that frameworks hide the backpropagation details behind a call such as `loss.backward()`.

== Takeaway

Backpropagation computes all parameter gradients efficiently by caching forward-pass values and reusing chain-rule computations backward. Initialization keeps activations and gradients from vanishing or exploding before learning begins.
