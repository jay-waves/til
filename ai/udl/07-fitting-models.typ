#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Book chapter 6],
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

= PDF 07: Fitting Models

== Training Problem

Previous chapters defined models and losses. Fitting a model means finding parameters that minimize the loss:

$ hat(phi) = argmin_(phi) L(phi) $

Most neural-network training algorithms are iterative:

1. initialize parameters
2. compute gradients of the loss with respect to parameters
3. update parameters to reduce the loss
4. repeat until training stops

This chapter focuses on step 3: algorithms that adjust parameters once gradients are available. Chapter 7 explains how to compute gradients efficiently and how to initialize parameters.

== Gradient Descent

For parameters $phi = [phi_0, phi_1, dots, phi_N]^T$, the gradient is:

$ (partial L) / (partial phi) = [(partial L)/(partial phi_0), (partial L)/(partial phi_1), dots, (partial L)/(partial phi_N)]^T $

The gradient points uphill in loss space. Gradient descent moves in the opposite direction:

$ phi_(t+1) = phi_t - alpha nabla_("phi") L(phi_t) $

The positive scalar $alpha$ controls the step size. If it is fixed, it is called the learning rate. Alternatively, a line search can try several step sizes and choose the one that decreases the loss most.

At a smooth local minimum, the gradient is zero. In practice, training may stop when the gradient magnitude is small, after a fixed number of iterations, or when validation performance stops improving.

== Linear Regression Example

For the 1D linear regression model:

$ f(x, phi) = phi_0 + phi_1 x $

with least-squares loss:

$ L(phi) = sum_(i=1)^I ell_i = sum_(i=1)^I (phi_0 + phi_1 x_i - y_i)^2 $

the gradient decomposes into a sum of per-example gradients:

$ (partial L) / (partial phi) = sum_(i=1)^I (partial ell_i) / (partial phi) $

where:

$ (partial ell_i) / (partial phi) = [2(phi_0 + phi_1 x_i - y_i), 2 x_i (phi_0 + phi_1 x_i - y_i)]^T $

For linear regression the loss is convex, so any downhill path with appropriate steps can reach the global minimum. This is not true for neural networks.

== Non-Convex Losses

The chapter uses a two-parameter Gabor model to visualize non-convex optimization:

$ f(x, phi) = sin(phi_0 + 0.06 phi_1 x) dot exp(-(phi_0 + 0.06 phi_1 x)^2 / 32.0) $

Unlike linear regression, this loss surface has multiple local minima and saddle points.

A local minimum has zero gradient and no nearby lower-loss direction, but it is not necessarily the global minimum. A saddle point has zero gradient but increases in some directions and decreases in others. Near saddle points, gradients can be small enough that a stopping rule may mistakenly treat training as converged.

Gradient descent is fully determined by initialization. If it starts in the wrong valley, it may converge to a poor local minimum, and there is no reliable local test that proves a better solution does not exist elsewhere.

== Stochastic Gradient Descent

Neural networks can have millions or billions of parameters, so exhaustive search and repeated full optimization from many initial points are not practical. Stochastic gradient descent (SGD) adds randomness by computing each update from a randomly selected minibatch.

A batch $B_t$ contains the indices used at iteration $t$:

$ phi_(t+1) = phi_t - alpha sum_(i in B_t) (partial ell_i(phi_t)) / (partial phi) $

This update descends the batch loss, not necessarily the full training loss. It can even move uphill with respect to the full loss, which is one reason SGD can escape some local minima or flat saddle regions.

Another view is that SGD performs deterministic gradient descent on a different loss function at every iteration. Despite the noisy path, the expected gradient matches the full-data gradient when batches are sampled appropriately.

== Batches, Epochs, and Schedules

A batch can contain one example, the whole dataset, or anything in between. Full-batch gradient descent is the special case where $B_t$ contains all training examples.

Batches are usually drawn without replacement until every example has been used. One pass through the entire dataset is an epoch.

SGD has several practical advantages:

- each update is cheaper than a full gradient
- every example contributes over an epoch
- noise can help escape local minima and saddle regions
- the noisy trajectory often finds solutions that generalize well

SGD often uses a learning-rate schedule. A larger early learning rate explores parameter space and can jump between valleys. A smaller later rate fine-tunes once training is in a useful region.

== Momentum

Momentum smooths SGD updates by combining the current batch gradient with a running direction:

$ m_(t+1) = beta m_t + (1 - beta) sum_(i in B_t) (partial ell_i(phi_t)) / (partial phi) $

$ phi_(t+1) = phi_t - alpha m_(t+1) $

The recursive definition makes $m_t$ a weighted average of past gradients. If gradients point in similar directions across many steps, the effective step grows. If gradients alternate, they cancel, reducing oscillation in narrow valleys.

== Nesterov Momentum

Nesterov accelerated momentum treats the momentum term as a prediction of the next location and evaluates the gradient there:

$ m_(t+1) = beta m_t + (1 - beta) sum_(i in B_t) (partial ell_i(phi_t - alpha beta m_t)) / (partial phi) $

$ phi_(t+1) = phi_t - alpha m_(t+1) $

This lets the gradient correct the path momentum would have taken alone, which can help when the loss valley bends.

== Adam

Fixed-step gradient descent struggles when the loss is steep in one parameter direction and shallow in another. A learning rate that is stable in the steep direction may be too slow in the shallow direction.

Adam adapts step sizes per coordinate by tracking a momentum-like estimate of the gradient and of the squared gradient:

$ m_(t+1) = beta m_t + (1 - beta) g_t $

$ v_(t+1) = gamma v_t + (1 - gamma) g_t^2 $

Because $m_t$ and $v_t$ start at zero, Adam applies bias correction:

$ tilde(m)_(t+1) = m_(t+1) / (1 - beta^(t+1)) $

$ tilde(v)_(t+1) = v_(t+1) / (1 - gamma^(t+1)) $

The update is:

$ phi_(t+1) = phi_t - alpha tilde(m)_(t+1) / (sqrt(tilde(v)_(t+1)) + epsilon) $

Adam is usually used with minibatches. It is less sensitive to raw gradient scale and often needs less careful learning-rate scheduling than plain SGD, though the effective learning rate per parameter is less transparent.

== Training Hyperparameters

The optimizer, learning rate, batch size, schedule, and momentum coefficients are training-algorithm hyperparameters. They directly affect final model performance but are not learned model parameters.

These choices are often selected empirically through hyperparameter search, discussed in the next chapter.

== Notes

Convexity can be checked through the Hessian, the matrix of second derivatives. For neural networks, losses are generally non-convex, so local curvature varies across parameter space.

Line search can choose a step size by probing the loss along the descent direction, but fixed or scheduled learning rates are more common for neural networks because line search is expensive.

== Takeaway

Model fitting minimizes a loss over parameters. Gradient descent follows the full gradient, SGD follows noisy minibatch gradients, momentum smooths the path, and Adam rescales updates using running gradient statistics.
