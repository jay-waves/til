#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Book chapter 17],
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

= PDF 18: Variational Autoencoders

== Motivation

VAEs are probabilistic latent-variable models. Like GANs, they generate by sampling latent variables and decoding them into data. Like flows, they optimize a likelihood-related objective and provide a principled density model, but they only optimize a lower bound.

== Latent Variable Model

A latent-variable model defines:

$ Pr(x) = integral Pr(x | z, phi) Pr(z) dif z $

The posterior is:

$ Pr(z | x, phi) = (Pr(x | z, phi) Pr(z)) / Pr(x) $

The denominator is usually intractable for nonlinear neural decoders.

Mixture of Gaussians is a simple latent-variable model: choose a component latent variable, then sample from that component's Gaussian. VAEs replace simple component distributions with neural decoders.

== Nonlinear Decoder and Generation

A neural latent-variable model samples $z$ from a simple prior, often $cal(N)(0, I)$, and uses a decoder network to parameterize $Pr(x | z, phi)$.

Generation is easy:

1. sample $z tilde Pr(z)$
2. compute decoder distribution $Pr(x | z, phi)$
3. sample or take a point estimate from that distribution

Training is hard because the marginal likelihood $Pr(x)$ requires integrating over all latent causes $z$.

== ELBO

The evidence lower bound uses a tractable distribution $q_theta(z | x)$ to bound the log likelihood:

$ log Pr(x) >= E_(q_theta(z | x)) [log Pr(x | z, phi)] - KL(q_theta(z | x) || Pr(z)) $

The first term rewards reconstruction under the decoder. The KL term keeps the approximate posterior close to the prior, so samples drawn from the prior decode to meaningful data.

The ELBO comes from Jensen's inequality. It replaces the intractable log of an integral with an expectation under $q_theta$.

== ELBO Properties

The ELBO is tight when the approximate posterior equals the true posterior:

$ q_theta(z | x) = Pr(z | x, phi) $

The gap between the log likelihood and the ELBO is:

$ KL(q_theta(z | x) || Pr(z | x, phi)) $

The ELBO can therefore be read either as reconstruction minus prior KL, or as true log likelihood minus posterior-approximation error.

== Variational Autoencoder

A VAE introduces an encoder network $g(x, theta)$ that predicts the parameters of $q_theta(z | x)$ and a decoder network parameterizing $Pr(x | z, phi)$.

This is amortized inference: instead of optimizing a separate posterior approximation for every training example, one shared encoder predicts the approximation from $x$.

The training loss is the negative ELBO. Optimizing it updates both encoder parameters $theta$ and decoder parameters $phi$.

== Reparameterization Trick

For Gaussian encoder:

$ q_theta(z | x) = cal(N)(mu_theta(x), diag(sigma_theta(x)^2)) $

Sample as:

$ z = mu_theta(x) + sigma_theta(x) dot epsilon, quad epsilon tilde cal(N)(0, I) $

This moves randomness to $epsilon$ and allows gradients through $mu_("theta")$ and $sigma_("theta")$.

Without reparameterization, sampling would block low-variance gradient flow through the encoder parameters.

== Using VAEs

VAEs support several operations:

- approximate sample probability with ELBO or importance sampling
- generation by sampling $z$ from the prior and decoding
- resynthesis by encoding an existing example and decoding it
- interpolation by moving through latent space
- representation learning from encoder outputs

== Disentanglement

VAEs can sometimes learn latent dimensions corresponding to interpretable factors, but this is not guaranteed. Beta-VAE and related methods modify the ELBO, often increasing the KL pressure, to encourage more factorized or disentangled latents.

The tradeoff is that stronger disentanglement pressure can reduce reconstruction fidelity.

== Practical Issues

VAEs can produce blurry samples when the decoder likelihood encourages averaging, such as Gaussian pixel likelihoods.

Posterior collapse occurs when the decoder ignores $z$ and the approximate posterior collapses toward the prior, especially with very powerful decoders.

Approximate posterior families may be too simple. Normalizing flows can be used inside VAEs to make the variational posterior more flexible.

== Notes

Diffusion models can be viewed as related to hierarchical latent-variable models. VAE ideas also connect to expectation-maximization: alternate improving latent explanations and model parameters, but VAEs amortize and differentiate this process.

== Takeaway

VAEs train neural latent-variable models by maximizing a tractable lower bound on log likelihood, using an encoder for approximate posterior inference and a decoder for generation.
