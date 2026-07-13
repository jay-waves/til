#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Book chapter 8],
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

= PDF 09: Measuring Performance

== Motivation

Training performance is not the same as real-world performance. A high-capacity neural network can often drive training error to zero while still failing on new examples.

This chapter asks how to measure generalization and explains why test errors arise from noise, bias, and variance. It also introduces hyperparameter selection and double descent.

== Training and Test Performance

The chapter uses MNIST-1D as an example. Inputs have $D_i = 40$ dimensions and labels are ten digit classes. A network with two hidden layers and softmax outputs is trained with multiclass cross-entropy.

The training error and training loss can fall to zero, but the test error remains high. This means the model has learned to classify the training set without learning a rule that transfers well to new samples.

The test loss can even increase while test error stays roughly constant. With softmax classification, the model may make the same wrong predictions with increasing confidence, assigning lower probability to the true class and increasing negative log-likelihood.

== Sources of Error

To analyze errors, the chapter switches to a 1D regression problem where the true data-generating function is known. The model is a simplified shallow ReLU network with fixed, evenly spaced joints and trainable output weights, so it can be fit in closed form.

There are three sources of expected test error:

- Noise: irreducible randomness or missing information in the data-generating process.
- Bias: systematic error because the model family cannot represent the true function.
- Variance: sensitivity of the learned function to the particular training sample.

Noise may come from genuine stochasticity, label errors, or unobserved explanatory variables. It limits test performance but may not prevent training error from reaching zero if exact inputs are rarely repeated.

Bias appears when even the best possible parameters in the model family cannot match the true function.

Variance appears because finite noisy datasets produce different fitted models. Stochastic optimization can add more variance if repeated training runs converge to different solutions.

== Bias-Variance Decomposition

For least-squares regression with additive noise, let the true conditional mean be:

$ mu(x) = E_y[y(x)] $

and noise variance:

$ sigma^2 = E_y[(mu(x) - y(x))^2] $

For a fitted model $f(x, phi[D])$ trained on random dataset $D$, define the average learned prediction:

$ f_mu(x) = E_D[f(x, phi[D])] $

The expected squared error decomposes as:

$ E_D E_y [L(x)] = E_D[(f(x, phi[D]) - f_mu(x))^2] + (f_mu(x) - mu(x))^2 + sigma^2 $

These terms are:

- variance: model changes caused by sampling a different training set
- squared bias: deviation of the average learned model from the true mean function
- noise: irreducible uncertainty in targets

For losses other than least squares, the same concepts exist but do not necessarily combine additively.

== Reducing Error

Noise is irreducible unless the data collection process changes, labels improve, or missing explanatory variables are added.

Variance can often be reduced by adding more training data. More examples average out noise and cover the input space better, making fitted models from different datasets more similar.

Bias can be reduced by increasing model flexibility, such as adding hidden units or layers. In the toy model, more hidden units create more linear regions and allow closer approximation of the true function.

== Bias-Variance Trade-Off

Increasing capacity usually reduces bias but can increase variance for a fixed training set. A low-capacity model underfits: it cannot match the true function. A high-capacity model may overfit: it uses flexibility to model noise in the training data.

In the classical picture, test error is minimized at an intermediate capacity where bias has fallen but variance has not yet grown too much.

== Double Descent

Modern overparameterized models often do not follow a simple U-shaped test-error curve. As capacity increases:

1. test error first decreases as bias falls
2. test error can rise near the interpolation threshold, where the model just has enough capacity to memorize the training data
3. test error can decrease again in the overparameterized regime

This pattern is double descent. The first part resembles the classical bias-variance trade-off; the second descent is the surprising modern-regime behavior.

The explanation is not settled. One plausible story is that once training data are interpolated, extra capacity changes behavior between training points. In high-dimensional spaces, data are extremely sparse, so interpolation behavior matters. Larger overparameterized models may be capable of smoother interpolations.

But capacity alone does not force smoothness. The smooth solution must be encouraged by some regularizer, explicit or implicit. Initialization and the training algorithm may bias learning toward smoother functions.

== Hyperparameter Selection

Model capacity, architecture, optimizer, learning rate, batch size, and schedule are hyperparameters. They are chosen outside the ordinary parameter-fitting process.

Hyperparameters should not be selected on the test set. Doing so leaks information from the test set into the modeling process, making the reported test performance optimistic.

Use three splits:

- training set: fit model parameters
- validation set: choose hyperparameters
- test set: final performance estimate after choices are fixed

Hyperparameter optimization is expensive because each candidate usually requires training a whole model. The space is often discrete, conditional, and too large for exhaustive search or gradient methods.

== Cross-Validation and Capacity Notes

When data are scarce, holding out a validation set can increase variance by reducing training data. $K$-fold cross-validation partitions the training/validation data into $K$ subsets, trains on $K - 1$ subsets, validates on the held-out subset, and averages validation performance across folds.

Capacity is informal in the chapter: number of parameters, hidden units, or ability to fit complex functions. More formal notions include representational capacity, effective capacity, VC dimension, and Rademacher complexity.

Representational capacity describes what the model family can express. Effective capacity also accounts for what the optimizer can actually reach.

== Takeaway

Performance is generalization. Test error comes from noise, bias, and variance; data, capacity, regularization, and hyperparameter selection determine how these errors appear in practice.
