#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Book chapter 16],
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

= PDF 17: Normalizing Flows

== Motivation

GANs can sample realistic data but do not provide tractable probabilities. Normalizing flows transform a simple base density into a complex data density using an invertible mapping, so they support both sampling and exact likelihood evaluation.

== Invertible Density Modeling

A flow maps latent variables to data:

$ x = f(z, phi), quad z tilde Pr_z $

Because $f$ is invertible:

$ z = f^(-1)(x, phi) $

The generative direction samples $z$ and maps it to $x$. The normalizing direction maps data $x$ back to a simple distribution over $z$.

== Change of Variables

$ Pr_x(x) = Pr_z(z) abs(det(J_(f^(-1))(x))) $

Log form:

$ log Pr_x(x) = log Pr_z(z) + log abs(det(J_(f^(-1))(x))) $

The determinant accounts for local volume expansion or contraction.

Training maximizes:

$ sum_i log Pr_x(x_i) $

which is tractable because $z_i = f^(-1)(x_i)$ and the Jacobian determinant are tractable by design.

== Layer Constraints

A practical flow layer needs easy forward mapping, easy inverse mapping, and cheap log determinant. An arbitrary expressive neural net is not enough.

Useful flow layers must balance:

- expressive transformation
- efficient inverse
- efficient determinant
- stable numerical behavior

== Linear and Elementwise Flows

A linear flow $f(h) = beta + Omega h$ is invertible if $Omega$ is invertible, but determinants and inverses are expensive in high dimensions unless $Omega$ has special structure.

Elementwise flows apply an invertible scalar function independently to each dimension. They are easy to invert and have diagonal Jacobians, but they cannot create dependencies between dimensions unless alternated with mixing layers.

== Coupling Flows

Split $h = (h_a, h_b)$:

$ h'_a = h_a $

$ h'_b = h_b dot exp(s(h_a)) + t(h_a) $

Inverse:

$ h_b = (h'_b - t(h_a)) dot exp(-s(h_a)) $

Log determinant:

$ sum_j s_j(h_a) $

Coupling layers are practical because their Jacobian is triangular. The conditioner networks $s$ and $t$ can be expressive even though the full transformation remains easy to invert.

== Autoregressive Flows

Autoregressive flows generalize coupling by making each output dimension depend on previous input dimensions. With the right masking, density evaluation can be efficient and the Jacobian is triangular.

Masked autoregressive flows are efficient in the normalizing direction but slow to sample because inverse generation is sequential. Inverse autoregressive flows reverse this tradeoff and are useful when fast sampling matters.

== Residual Flows

Residual flows use residual-style transformations that remain invertible. iRevNet splits channels and applies invertible residual-like updates. iResNet uses contraction mappings so the inverse exists and can be found iteratively.

These designs increase architectural flexibility but often complicate inversion or determinant estimation.

== Multi-Scale Flows

Flows require latent and data spaces to have the same total dimension. Multi-scale flows partition the latent variables across stages. Some components are factored out early, while the rest continue through deeper transformations.

This reduces computation and lets different latent variables represent different levels of detail.

== Applications

Flows are useful for:

- density estimation and anomaly detection
- sampling and image synthesis
- interpolation in latent space
- approximating other distributions, such as posteriors in VAEs

== Limitations

Flows preserve dimensionality and must use invertible layers. This can be awkward when data lie near a lower-dimensional manifold or when the most natural architecture is not invertible.

Their samples are often weaker than GANs or diffusion models, though likelihood evaluation is much more direct.

== Notes

GLOW is a well-known multi-scale flow for images. It uses invertible convolutional and coupling-style operations and can synthesize and interpolate images, though more recent GANs and diffusion models usually produce higher visual quality.

Normalizing flows are also used as flexible variational distributions, improving posterior approximations in latent-variable models.

== Takeaway

Normalizing flows transform a tractable base density through invertible layers. They trade architectural freedom for exact likelihood, exact latent inference, and reversible sampling.
