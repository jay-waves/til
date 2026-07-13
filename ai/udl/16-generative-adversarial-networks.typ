#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Book chapter 15],
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

= PDF 16: Generative Adversarial Networks

== Motivation

GANs are implicit generative models. A generator maps latent noise to data-like samples, but the model does not directly provide a likelihood. Instead, it learns through a discriminator that tries to distinguish real from generated examples.

== Generator and Discriminator

Generator:

$ x = g(z, phi), quad z tilde Pr_z $

Discriminator:

$ d(x, theta) in [0, 1] $

The discriminator learns a loss signal by separating real and generated data. The generator improves by exploiting that signal.

== GAN Objective

$ min_("phi") max_("theta") E_(x tilde Pr_("data")) log d(x, theta) + E_(z tilde Pr_z) log(1 - d(g(z, phi), theta)) $

The discriminator parameters $theta$ are trained to classify real examples as real and generated examples as fake. The generator parameters $phi$ are trained to make generated examples look real.

The non-saturating generator loss is often used:

$ L_G(phi) = - E_(z tilde Pr_z) log d(g(z, phi), theta) $

It provides stronger gradients when the discriminator is confident.

For a fixed generator, the optimal discriminator estimates where the data distribution has more mass than the generator distribution. The original GAN objective can be related to Jensen-Shannon divergence.

== DCGAN and Image Generation

DCGAN adapted GANs to images with convolutional generators and discriminators. The generator starts from a latent vector and upsamples through convolutional structure. The discriminator downsamples images and classifies real versus generated.

DCGAN required careful architecture and training heuristics: convolutional layers, batch normalization, ReLU-like activations, and avoiding unstable design choices.

== Training Instability

The target changes because both networks learn. Common problems:

- vanishing gradients when the discriminator is too strong
- mode collapse when the generator covers only a few modes
- oscillation between generator and discriminator strategies
- high sensitivity to architecture and hyperparameters

Mode collapse is especially important: the generator may produce realistic-looking samples from only a small part of the data distribution.

== Wasserstein Distance

The original GAN loss can provide weak gradients when real and generated distributions have little overlap. Wasserstein or earth mover's distance measures the minimum cost of transporting mass from one distribution to another. It remains meaningful even when supports do not overlap.

== Wasserstein GAN

Wasserstein GAN replaces the discriminator with a critic whose output is not a probability. The critic estimates a quantity related to Wasserstein distance. The generator minimizes this distance.

The critic must satisfy a Lipschitz constraint. Early WGAN used weight clipping; WGAN-GP adds a gradient penalty; spectral normalization is another common approach.

== Stabilization Tricks

Progressive growing trains low-resolution generation first and gradually adds layers for higher resolution.

Minibatch discrimination gives the discriminator access to relationships among generated samples, helping detect lack of diversity.

Truncation samples latent variables from a smaller high-probability region. It improves average sample quality but reduces diversity.

== Conditional and Translation Models

Conditional GANs generate with side information:

$ x = g(z, c, phi) $

Conditional GANs pass condition $c$ to generator and discriminator. Auxiliary classifier GANs add a classifier objective so generated samples match requested classes. InfoGAN encourages interpretable latent factors by maximizing information between selected latent variables and generated samples.

== Image Translation

Pix2Pix uses paired image translation: each input image has a corresponding target image. It combines a supervised reconstruction loss with an adversarial loss so outputs are both close to the target and realistic.

CycleGAN handles unpaired domains. It trains mappings in both directions and adds cycle consistency:

$ x arrow g(x) arrow f(g(x)) approx x $

This discourages arbitrary mappings that fool the discriminator but do not preserve image content.

SRGAN applies adversarial loss to super-resolution, encouraging perceptually realistic high-frequency detail rather than only minimizing pixel error.

== StyleGAN

StyleGAN separates coarse and fine variation more explicitly. It maps the latent vector into an intermediate style space and injects style information at multiple scales. It also injects noise into the generator at different layers for stochastic details.

This architecture enables high-quality image synthesis and controllable latent-space editing.

== Notes

GANs have been most successful for images, but the same adversarial idea has been applied to graphs, audio, image editing, and domain translation.

GAN inversion tries to find a latent code that reconstructs a real image, enabling editing through latent manipulation. This is harder than in VAEs or flows because a basic GAN has no encoder.

The central strength of GANs is sharp sample quality. The central weakness is fragile game-like training and incomplete coverage of the data distribution.

== Takeaway

GANs train a generator with a learned adversarial loss. They can produce sharp realistic samples and support conditional generation, but training is unstable and mode coverage is difficult to guarantee.
