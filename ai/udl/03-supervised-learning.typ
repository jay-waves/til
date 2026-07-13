#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Book chapter 2],
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

= PDF 03: Supervised Learning

== Supervised Learning Overview

A supervised learning model maps one or more inputs to one or more outputs. In the Prius example from the chapter, the input could contain the age and mileage of a car, and the output could be the estimated dollar value.

For this introductory setup, both $x$ and $y$ are fixed-size vectors with a fixed ordering of their elements. This is structured or tabular data: each coordinate has a stable meaning across examples.

The model is a mathematical function. Running the function on an input to produce an output is inference:

$ y = f(x) $

The function contains parameters $phi$, so it really describes a family of possible input-output relationships:

$ y = f(x, phi) $

Different parameter values choose different members of the family. Learning or training means selecting parameters that make the model predict the training outputs from the corresponding training inputs.

The training dataset contains $I$ paired examples:

$ cal(D) = { (x_i, y_i) }_(i=1)^I $

The loss $L$ is a scalar summary of mismatch between predictions and targets. Strictly, the loss depends on both the dataset and the parameters, but the chapter writes $L(phi)$ when the dataset is fixed:

$ hat(phi) = argmin_(phi) L(phi) $

The loss surface is therefore a function of parameters, not of individual inputs. The data define the surface; optimization moves through parameter space.

After training, performance must be assessed on separate test data. Training loss measures fit to observed examples; test loss estimates whether the fitted relationship generalizes to new inputs.

== 1D Linear Regression Model

The chapter makes the supervised-learning pipeline concrete with a scalar input and scalar output. A 1D linear regression model predicts with a straight line:

$ y = f(x, phi) = phi_0 + phi_1 x $

The two parameters have direct geometric meanings:

- $phi_0$: y-intercept
- $phi_1$: slope

Changing $phi_0$ or $phi_1$ changes the line, so equation 2.4 defines the family of all possible lines, and a particular parameter vector chooses one line.

== Least-Squares Loss

For each training pair, the residual or prediction error is:

$ r_i = f(x_i, phi) - y_i = phi_0 + phi_1 x_i - y_i $

The chapter defines the total mismatch as the sum of squared residuals:

$ L(phi) = sum_(i=1)^I (f(x_i, phi) - y_i)^2 = sum_(i=1)^I (phi_0 + phi_1 x_i - y_i)^2 $

This is a least-squares loss. Squaring has two immediate effects:

- errors above and below the line do not cancel
- larger errors are penalized disproportionately

Chapter 5 later gives the probabilistic reason for this choice: least squares is equivalent to maximum likelihood under fixed-variance Gaussian observation noise.

Every parameter pair $(phi_0, phi_1)$ defines one line and one loss value. The set of all values forms a loss surface or cost function over parameter space. In the chapter's figure, poorly fitting lines sit at high-loss locations, while the best-fitting green line sits at the minimum.

The fitted parameters are:

$ hat(phi) = argmin_(phi) [sum_(i=1)^I (f(x_i, phi) - y_i)^2] $

or, substituting the line equation:

$ hat(phi) = argmin_(phi) [sum_(i=1)^I (phi_0 + phi_1 x_i - y_i)^2] $

== Gradients and Closed-Form Solution

The gradient components are:

$ (partial L) / (partial phi_0) = 2 sum_(i=1)^I (phi_0 + phi_1 x_i - y_i) $

$ (partial L) / (partial phi_1) = 2 sum_(i=1)^I x_i (phi_0 + phi_1 x_i - y_i) $

Setting these to zero gives the normal equations and a closed-form solution for $phi_0$ and $phi_1$. The chapter emphasizes that this convenience is special to simple linear regression. More complex models usually have no practical closed-form solution and too many parameters to search exhaustively.

== Training

Model fitting, training, and learning all refer to the process of finding low-loss parameters. The basic iterative idea is:

1. initialize parameters, often randomly
2. compute the loss and its gradient at the current parameters
3. step downhill in parameter space
4. repeat until improvement stalls

A gradient descent step has the form:

$ phi arrow phi - alpha nabla_("phi") L(phi) $

where $alpha$ is the step size. In the chapter's figure, each position on the loss-contour plot corresponds to a different line in data space; as the optimizer moves downhill, the corresponding line fits the points more closely.

== Testing, Underfitting, and Overfitting

The purpose of testing is to estimate real-world performance. The test set must be separate from training because the model has already been optimized to explain the training examples.

Generalization depends on both the data and the model family:

- If the training set is incomplete or unrepresentative, low training loss may not transfer.
- If the model is too simple, it may underfit by failing to capture the true relationship.
- If the model is too expressive, it may overfit by capturing accidental peculiarities of the training set.

A straight line is easy to fit and visualize, but it can only represent linear relationships. This motivates shallow and deep neural networks in the next chapters.

== Discriminative vs. Generative

Discriminative model:

$ y = f(x, phi) $

Generative model:

$ x = g(y, phi) $

The models in the chapter are discriminative: they directly predict $y$ from real-world measurements $x$. A generative model instead describes how measurements $x$ could be produced from hidden or target variables $y$.

Generative models can include prior knowledge about the data-creation process, such as geometry, object shape, or lighting. Their drawback is inference: to predict $y$ from $x$, one must invert or approximate the inverse of $g$, which may be difficult.

Modern machine learning is dominated by flexible discriminative models trained with large datasets. In many settings this beats the advantage of manually encoding prior knowledge in a generative model.

== Terminology Notes

The chapter notes a slight distinction between loss and cost. A loss can mean the per-example term, while a cost is the full quantity minimized over the dataset and possibly additional regularization terms. In practice, the book often uses loss function and cost function interchangeably. Objective function is the more general term for any function to be minimized or maximized.

The supervised learning template established here is:

- choose a parameterized model family
- choose a loss that measures task-relevant mismatch
- optimize parameters on training data
- estimate generalization with held-out data
- deploy only if the test distribution is a good proxy for future inputs

== Takeaway

Supervised learning fits a parameterized function $y = f(x, phi)$ by minimizing a loss on paired examples, then tests whether the learned relationship generalizes beyond the training set.
