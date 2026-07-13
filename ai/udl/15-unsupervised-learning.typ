#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Book chapter 14],
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

= PDF 15: Unsupervised Learning

== Goal

Unsupervised learning extracts structure from data without labels. The following chapters focus on generative modeling: learning a model of the data distribution $Pr(x)$ or a way to sample from it.

Unsupervised learning can support density estimation, sample generation, representation learning, clustering, compression, anomaly detection, and data synthesis. The book focuses on generative models.

== Taxonomy

The chapter groups unsupervised models into approaches that learn from unlabeled examples $x_i$.

Some models learn representations or latent variables $z$ that summarize important structure. Others learn to generate new examples $x^*$ from random latent variables.

The next chapters cover four major generative-model families:

- GANs: learn to generate samples using an adversarial discriminator signal
- normalizing flows: learn invertible maps with exact likelihood
- VAEs: learn latent-variable models by maximizing an evidence lower bound
- diffusion models: learn iterative denoising from noise to data

== What Makes a Good Generative Model?

Generative models based on latent variables should ideally:

- generate samples that look like real data
- cover all important modes of the true data distribution
- provide useful latent representations
- allow sampling in practical time
- support likelihood or probability evaluation when the application needs it

These goals can conflict. Sharp visual samples do not imply good likelihood or full mode coverage. High likelihood does not guarantee appealing samples.

== Fitting Generative Models

The model families use different training signals:

- GANs use a learned discriminator because no explicit likelihood is available.
- Flows maximize exact likelihood using the change-of-variables formula.
- VAEs maximize a lower bound on likelihood because exact marginal likelihood is intractable.
- Diffusion models use a variational or denoising objective across noise levels.

The families make different tradeoffs. GANs can generate sharp images but are difficult to train and can drop modes. Flows give exact likelihood and latent inference but restrict architecture to invertible maps. VAEs give a principled latent-variable model but can produce blurry samples. Diffusion models train stably and generate high-quality samples but often sample slowly.

== Evaluation Axes

Generative models must be evaluated along multiple axes:

- fidelity: do samples look real?
- diversity: are all modes covered?
- likelihood: does the model assign high probability to real data?
- latent structure: is the representation useful?
- sampling speed: how expensive is generation?

Likelihood and visual quality can disagree, so no single metric is sufficient.

== Inception Score and FID

For image generation, some metrics use a pretrained classifier or feature network.

The Inception Score rewards samples that are confidently classified while also being diverse across classes. It can miss failures within a class and does not directly compare generated samples to real examples.

Fréchet Inception Distance (FID) embeds real and generated images in feature space, fits Gaussian summaries to both sets, and measures the distance between the two Gaussians. It compares generated data to real data but depends on the feature extractor and only captures summary statistics.

== Manifold Precision and Recall

Generative precision measures whether generated samples lie near the real-data manifold. Generative recall measures whether real examples are covered by the generated-sample manifold.

High precision with low recall indicates mode collapse: generated samples look realistic but cover only part of the data distribution.

High recall with low precision indicates broad but unrealistic sampling: many real examples are near generated regions, but many generated samples are not realistic.

== Evaluation Principles

Evaluation should match the model's intended use. A density model needs probability estimates. A generator for creative images needs sample quality and coverage. A representation learner needs useful downstream embeddings.

Visual inspection remains useful but insufficient. Metrics can also be gamed or can miss perceptually important errors.

== Takeaway

Unsupervised generative modeling learns structure from unlabeled data. Good models should generate realistic samples, cover the data distribution, and support the kind of probability, representation, or sampling behavior required by the task.
