#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Book chapter 4],
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

= PDF 05: Deep Neural Networks

== Motivation

Shallow networks with enough hidden units can approximate continuous functions, but some functions require an impractically large number of hidden units. Deep neural networks add more than one hidden layer. With ReLU activations they still describe piecewise-linear mappings, but they can create many more linear regions for a given number of parameters.

Depth matters because layers compose functions. Earlier layers can transform, partition, or fold the input space; later layers operate on the transformed representation. When unfolded back to the original input, later computations can be replicated across many regions.

== Composing Shallow Networks

The chapter first considers two shallow scalar networks. The first maps $x$ to $y$:

$ h_1 = a(theta_(1 0) + theta_(1 1) x) $

$ h_2 = a(theta_(2 0) + theta_(2 1) x) $

$ h_3 = a(theta_(3 0) + theta_(3 1) x) $

$ y = phi_0 + phi_1 h_1 + phi_2 h_2 + phi_3 h_3 $

The second maps $y$ to $y'$ through another set of hidden units:

$ h'_e = a(theta'_(e 0) + theta'_(e 1) y) $

$ y' = phi'_0 + phi'_1 h'_1 + phi'_2 h'_2 + phi'_3 h'_3 $

If the first network maps several different input intervals of $x$ onto the same range of $y$, then the second network applies the same piecewise-linear computation to each folded interval. In the chapter's example, three regions from the first network and three regions from the second can compose into nine regions.

The same idea extends to higher-dimensional inputs. A first network can produce several non-flat regions, and a second network can subdivide each of those regions. This gives the folding interpretation of deep networks: earlier layers fold the input space, and later layers process the folded copies.

== From Composition to a Two-Layer Network

Composing two shallow networks is a special case of a network with two hidden layers. Substituting the first network's output into the second network's pre-activations gives:

$ h'_e = a(psi_(e 0) + sum_(d=1)^3 psi_(e d) h_d) $

where the $psi$ parameters combine the second network's input weights with the first network's output weights.

A general two-hidden-layer ReLU network is:

$ h_d = a(theta_(d 0) + theta_(d 1) x) $

$ h'_e = a(psi_(e 0) + sum_(d=1)^D psi_(e d) h_d) $

$ y' = phi'_0 + sum_(e=1)^D phi'_e h'_e $

This family is broader than literal composition of two scalar shallow networks because the matrix of $psi_(e d)$ values can be arbitrary; in the pure composition derivation, those values are constrained by an outer-product structure.

== Clipping View of Deep Networks

The chapter gives a second interpretation besides folding:

1. The first hidden layer computes ordinary ReLU hidden units from the input.
2. Linear combinations of those hidden units form piecewise-linear pre-activations for the next layer.
3. The next ReLU layer clips these already piecewise-linear functions, adding new joints.
4. The output layer recombines the clipped functions.

The folding view emphasizes replicated dependencies across regions. The clipping view emphasizes how new joints and regions are created layer by layer. Both are partial intuitions; the network is ultimately one nested equation mapping $x$ to $y$.

== Hyperparameters

Deep networks can have many hidden layers. The number of hidden layers is the depth, and the number of hidden units in each layer is the width. The total number of hidden units is a measure of capacity.

If the network has $K$ hidden layers with widths $D_1, D_2, dots, D_K$, then $K$ and the $D_k$ values are hyperparameters. They are chosen before training. The learned weights and biases are parameters.

For fixed hyperparameters, the architecture defines a family of functions. Varying hyperparameters gives a family of families of functions.

== Matrix Notation

For $K$ hidden layers:

$ h_1 = a(beta_0 + Omega_0 x) $

$ h_k = a(beta_(k - 1) + Omega_(k - 1) h_(k - 1)), quad k = 2, dots, K $

$ y = beta_K + Omega_K h_K $

The parameter set is:

$ phi = { beta_k, Omega_k }_(k=0)^K $

If $x in RR^(D_i)$, $y in RR^(D_o)$, and hidden layer $k$ has $D_k$ units, then:

- $beta_(k-1)$ has length $D_k$
- $beta_K$ has length $D_o$
- $Omega_0$ has shape $D_1 times D_i$
- $Omega_K$ has shape $D_o times D_K$
- intermediate $Omega_k$ has shape $D_(k+1) times D_k$

The same model can be written as one nested function:

$ y = beta_K + Omega_K a(beta_(K - 1) + Omega_(K - 1) a(dots a(beta_0 + Omega_0 x) dots)) $

Matrix notation is the practical language for large networks because writing scalar equations quickly becomes unreadable.

== Shallow vs. Deep Networks

Both shallow and deep networks can approximate continuous functions with enough capacity. A deep network can imitate a shallow network if later layers learn identity-like transformations.

The difference is efficiency and structure. A shallow scalar ReLU network with $D > 2$ hidden units can create at most $D + 1$ linear regions using $3 D + 1$ parameters. A scalar network with $K$ hidden layers of width $D > 2$ can create up to:

$ (D + 1)^K $

linear regions using:

$ 3 D + 1 + (K - 1) D (D + 1) $

parameters.

Deep networks can therefore create far more regions per parameter. This is not automatically useful: the regions are not independent; they contain dependencies and symmetries created by folding. The advantage is strongest when real tasks have similar compositional structure.

== Depth Efficiency

Depth efficiency means that some functions can be approximated by deep networks with far fewer units than any comparable shallow network. Some theoretical constructions require exponentially more hidden units in a shallow network to match a deep representation.

This is an attractive explanation for why depth helps, but it does not prove that every real-world target function has the required structure.

== Large Structured Inputs

Fully connected networks are inefficient for large structured inputs such as images with millions of pixels. They would have too many parameters, and they would relearn the same local patterns independently at every position.

Large structured inputs are better handled by local-to-global multi-stage processing: process nearby variables first, then progressively integrate information from wider regions. This is difficult to express without multiple layers and motivates convolutional architectures.

== Training and Generalization

Moderately deep networks can be easier to fit than very wide shallow ones, possibly because overparameterized deep models contain many roughly equivalent low-loss solutions. But adding too many layers makes training hard again because signals and gradients can degrade.

Deep networks also tend to generalize better in practice, and many top-performing models use tens or hundreds of layers. The reasons are not fully understood. Later chapters return to initialization, optimization, regularization, and residual connections as practical methods for making depth usable.

== Notes

The modern era of deep learning was enabled by several factors arriving together: larger datasets, more compute, ReLU activations, and stochastic gradient descent.

For ReLU networks with a total of $D$ hidden units, a general upper bound on the number of regions is $2^D$. Tighter bounds depend on input dimension, number of layers, and width per layer.

The depth version of universal approximation says that sufficiently deep ReLU networks with enough width can approximate broad classes of functions. Width-efficiency results ask the complementary question: when can wide shallow networks be replaced by narrower deeper networks?

The main point is not that depth magically improves every model, but that it changes the representation budget: repeated composition can be much more efficient than one large hidden layer.

== Takeaway

Deep networks represent functions by repeated composition. Each layer can fold, clip, and recombine representations, allowing many more structured linear regions per parameter than a shallow network.
