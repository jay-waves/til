#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Book chapter 18],
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

= PDF 19: Diffusion Models

== Motivation

Diffusion models are generative models that learn to reverse a gradual noising process. Instead of producing data in one step, they start from noise and iteratively denoise toward the data distribution.

== Forward Process

Diffusion chooses a fixed noising process:

$ q(z_t | z_(t - 1)) = cal(N)(sqrt(1 - beta_t) z_(t - 1), beta_t I) $

Let:

$ alpha_t = 1 - beta_t, quad bar(alpha)_t = product_(s=1)^t alpha_s $

Then:

$ q(z_t | x) = cal(N)(sqrt(bar(alpha)_t) x, (1 - bar(alpha)_t) I) $

or:

$ z_t = sqrt(bar(alpha)_t) x + sqrt(1 - bar(alpha)_t) epsilon, quad epsilon tilde cal(N)(0, I) $

As $t$ increases, $z_t$ becomes less like the data and more like Gaussian noise. The closed-form expression $q(z_t | x)$ lets training sample any timestep directly rather than simulating every noising step.

== Reverse Process

Generation learns denoising transitions:

$ p_("theta")(z_(t - 1) | z_t) $

Sampling starts from Gaussian noise and repeatedly applies the learned reverse process.

The reverse process is learned because moving from noise back toward data requires knowledge of the data distribution.

== Training and ELBO

Diffusion models can be treated as latent-variable models with latent chain $z_1, dots, z_T$. The forward process is the encoder and is fixed; the learned reverse process is the decoder.

The log likelihood is intractable, so training maximizes a variational lower bound. For Gaussian forward and reverse transitions, many terms simplify because the relevant conditionals are Gaussian. This leads to objectives that compare the true reverse conditional from the forward process with the learned reverse conditional.

In practice, the ELBO-derived loss is usually simplified to a denoising regression problem, which is easier to implement and works well.

== Noise Prediction Objective

A common simplified objective is:

$ L(theta) = E_(x, epsilon, t) norm(epsilon - epsilon_("theta")(z_t, t))_2^2 $

This is derived from the ELBO for Gaussian reverse transitions. Predicting noise, clean data, or velocity are related parameterizations.

The model receives the noisy sample $z_t$ and a timestep embedding. The timestep is necessary because the same noisy value can require different denoising behavior depending on the noise level.

== Architecture and Conditioning

Image diffusion commonly uses U-Nets with time embeddings, residual blocks, and attention. Conditional generation uses labels, text embeddings, or guidance:

$ epsilon_("theta")(z_t, t, c) $

Classifier-free guidance combines conditional and unconditional predictions, improving condition adherence at the cost of diversity.

== Sampling

Sampling begins with $z_T tilde cal(N)(0, I)$ and repeatedly applies the learned reverse transitions until $z_0$ is reached.

DDPM-style sampling is stochastic and can require many neural-network evaluations. Accelerated samplers and deterministic variants reduce the number of steps, trading speed, quality, and diversity.

== Relation to Score Matching

Noise prediction can be interpreted as learning the direction from noisy points back toward higher-density data regions. This connects diffusion models to denoising score matching.

== Latent and Conditional Diffusion

Pixel-space diffusion can be expensive. Latent diffusion first compresses images with an autoencoder and runs diffusion in the latent space, then decodes back to pixels.

Conditioning can use class labels, text embeddings, segmentation masks, low-resolution images, or other modalities. Text-to-image systems combine a text encoder with a conditional denoising model.

Guidance strengthens adherence to the condition by moving samples more toward the conditional prediction than the unconditional one. Strong guidance can improve prompt following but reduce diversity or create artifacts.

== Strengths and Limits

Diffusion models are stable to train and produce high-quality, diverse samples. The main cost is slow generation because sampling requires many denoising steps.

Performance depends on the noise schedule, parameterization, architecture, conditioning method, and sampler.

Generation speed can be improved by using fewer timesteps, deterministic samplers, distillation, or improved parameterizations. Generation quality can be improved with stronger architectures, classifier or classifier-free guidance, cascaded models, and operating in latent spaces.

== Takeaway

Diffusion models define a fixed noising process and learn its reverse. Generation starts from Gaussian noise and repeatedly denoises, often with U-Net-like architectures and optional conditioning.
