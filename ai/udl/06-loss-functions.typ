#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Book chapter 5],
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

= PDF 06: Loss Functions

== Motivation

The previous chapters defined model families: linear regression, shallow networks, and deep networks. Training now requires defining what "best possible mapping" means for a dataset $cal(D) = { (x_i, y_i) }_(i=1)^I$.

A loss or cost function $L(phi)$ returns a scalar mismatch between model predictions and ground-truth outputs. The parameters are trained by minimizing this scalar.

The chapter's key move is probabilistic: instead of treating the network as directly predicting $y$, treat it as predicting parameters of a conditional distribution over possible outputs:

$ Pr(y | x) = Pr(y | f(x, phi)) $

The observed target should have high probability under the distribution predicted from the corresponding input.

== Maximum Likelihood

To build a distributional model:

1. Choose a parametric distribution $Pr(y | theta)$ whose domain matches the prediction target.
2. Let the neural network compute one or more distribution parameters $theta_i = f(x_i, phi)$.
3. Choose parameters $phi$ that make the training outputs likely.

The likelihood of all training outputs is:

$ product_(i=1)^I Pr(y_i | x_i, phi) $

or, emphasizing the network-produced distribution parameters:

$ product_(i=1)^I Pr(y_i | f(x_i, phi)) $

This factorization assumes the examples are independent and identically distributed: each conditional output distribution has the same form, and the outputs are independent given their inputs.

Maximum likelihood chooses:

$ hat(phi) = argmax_(phi) [product_(i=1)^I Pr(y_i | f(x_i, phi))] $

Products of many probabilities can underflow. Since $log$ is monotonically increasing, maximizing likelihood is equivalent to maximizing log-likelihood:

$ hat(phi) = argmax_(phi) [sum_(i=1)^I log Pr(y_i | f(x_i, phi))] $

By convention, training is framed as minimization, so the loss is the negative log-likelihood:

$ L(phi) = - sum_(i=1)^I log Pr(y_i | x_i, phi) $

Inference can return the full predicted distribution, but often returns the most probable output:

$ hat(y) = argmax_(y) Pr(y | f(x, hat(phi))) $

== Recipe for Loss Construction

1. Choose a distribution matching the output type.
2. Let the network predict the distribution parameters.
3. Evaluate the observed target under that distribution.
4. Minimize negative log-likelihood over the training set.
5. At inference, return either the full distribution or a point estimate.

The loss is therefore not arbitrary: it encodes an assumption about output noise or class uncertainty.

== Univariate Regression

For scalar regression, choose a univariate normal distribution over $y in RR$:

$ Pr(y | mu, sigma^2) = 1 / sqrt(2 pi sigma^2) exp(-(y - mu)^2 / (2 sigma^2)) $

Let the model predict the mean:

$ mu_i = f(x_i, phi) $

Then the negative log-likelihood is:

$ L(phi) = - sum_(i=1)^I log Pr(y_i | f(x_i, phi), sigma^2) $

After removing terms that do not depend on $phi$ and removing positive constant scaling, the optimizer is unchanged and the objective becomes least squares:

$ L(phi) prop sum_i (y_i - f(x_i, phi))^2 $

Thus least squares follows from assuming independent Gaussian targets with mean predicted by the model and fixed variance.

For inference, the normal distribution's maximum is at its mean, so:

$ hat(y) = f(x, hat(phi)) $

== Estimating Uncertainty

The fixed-variance least-squares objective does not depend on $sigma^2$, but one can also learn a global variance by minimizing the full negative log-likelihood with respect to both $phi$ and $sigma^2$.

If uncertainty changes with the input, the problem is heteroscedastic. A network can predict both the mean and variance:

$ mu = f_1(x, phi) $

$ sigma^2 = f_2(x, phi)^2 $

The square ensures the variance is positive. The resulting loss contains both an error term and a variance term. Predicting a large variance can reduce the residual penalty, but it is penalized by the normalizing term in the log-likelihood.

Homoscedastic regression assumes constant uncertainty. Heteroscedastic regression lets the predicted uncertainty vary with $x$.

== Binary Classification

Binary classification predicts labels $y in {0, 1}$. Choose a Bernoulli distribution:

$ Pr(y | lambda) = (1 - lambda)^(1 - y) dot lambda^y $

where $lambda$ is the probability of class 1. Since $lambda$ must lie in $[0, 1]$, pass the real-valued network output through the logistic sigmoid:

$ lambda = sigmoid(f(x, phi)) = 1 / (1 + exp(-f(x, phi))) $

The likelihood for an example is:

$ Pr(y_i | x_i, phi) = (1 - lambda_i)^(1 - y_i) dot lambda_i^(y_i) $

The negative log-likelihood is binary cross-entropy:

$ L(phi) = - sum_i ((1 - y_i) log(1 - lambda_i) + y_i log lambda_i) $

For a point prediction, classify as $1$ when $lambda > 0.5$ and $0$ otherwise.

== Multiclass Classification

Multiclass classification predicts $y in {1, 2, dots, K}$. Choose the categorical distribution with class probabilities $lambda_1, dots, lambda_K$, all non-negative and summing to one.

Use a network with $K$ outputs and transform them with softmax:

$ p_k = softmax(z)_k = exp(z_k) / sum_(m=1)^K exp(z_m) $

where $z = f(x, phi)$.

The likelihood for a target class $y_i$ is the softmax probability assigned to that class:

$ Pr(y_i | x_i, phi) = softmax_(y_i)(f(x_i, phi)) $

The negative log-likelihood is multiclass cross-entropy:

$ L(phi) = - sum_i log softmax_(y_i)(f(x_i, phi)) $

Equivalently, for logits $f_k(x_i, phi)$:

$ L(phi) = - sum_i (f_(y_i)(x_i, phi) - log sum_(k=1)^K exp(f_k(x_i, phi))) $

For a point prediction, choose the class with the largest predicted probability. Implementations usually combine softmax and log-loss for numerical stability rather than computing probabilities first.

== Other Prediction Domains

The same recipe applies beyond ordinary regression and classification. The distribution should match the target support:

- real-valued scalar: normal, Laplace, Student's t, or mixture of Gaussians
- positive magnitude: exponential or gamma
- bounded proportion: beta
- multivariate real vector: multivariate normal
- circular angle: von Mises
- binary label: Bernoulli
- multiclass label: categorical
- count: Poisson
- ranking or permutation: Plackett-Luce

Choosing a distribution is part of modeling. It encodes what kinds of outputs are possible and what errors should be considered plausible.

== Multiple Outputs

When the target $y$ is a vector, it is common to assume output dimensions are conditionally independent given the network output:

$ Pr(y | f(x, phi)) = product_d Pr(y_d | f_d(x, phi)) $

Then the negative log-likelihood becomes a sum across both examples and output dimensions:

$ L(phi) = - sum_(i=1)^I sum_d log Pr(y_(i d) | f_d(x_i, phi)) $

This handles multi-output regression, dense classification such as one class per pixel, or mixed prediction tasks.

For mixed outputs, choose one distribution per component. For example, wind prediction could use a von Mises distribution for direction and an exponential distribution for positive speed. Under the independence assumption, the joint likelihood is a product, so the loss is the sum of the component negative log-likelihoods.

== Cross-Entropy

Cross-entropy gives another route to the same objective. Let $q(y)$ be the empirical distribution of observed targets and $p(y) = Pr(y | theta)$ be the model distribution. Minimizing KL divergence:

$ KL(q || p) $

with respect to model parameters is equivalent to dropping the part independent of the model and minimizing:

$ H(q, p) = - sum_y q(y) log p(y) $

For empirical point-mass targets, this reduces to:

$ - sum_i log Pr(y_i | theta) $

When $theta = f(x_i, phi)$, this is exactly the negative log-likelihood criterion. Thus maximum likelihood and cross-entropy minimization are equivalent perspectives.

== Takeaway

Loss functions are probability-modeling assumptions written as optimization objectives: choose a distribution for the target, let the network predict its parameters, and minimize the negative log probability of the observed data.
