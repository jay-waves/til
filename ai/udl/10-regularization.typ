#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Book chapter 9],
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

= PDF 10: Regularization

== Motivation

Chapter 8 showed that training and test performance can differ substantially. A model may overfit statistical peculiarities of the training data, or it may behave poorly in regions with no examples.

Regularization broadly means any strategy that improves generalization. Strictly, it means adding an explicit term to the loss that favors some parameter settings over others. The chapter covers both strict explicit regularization and broader implicit or heuristic methods.

== Explicit Regularization

For training examples $(x_i, y_i)$ and per-example losses $ell_i$, ordinary training solves:

$ hat(phi) = argmin_(phi) [sum_(i=1)^I ell_i(x_i, y_i)] $

Explicit regularization adds a scalar penalty $g(phi)$:

$ hat(phi) = argmin_(phi) [sum_(i=1)^I ell_i(x_i, y_i) + lambda g(phi)] $

The coefficient $lambda > 0$ controls the tradeoff between fitting the training data and satisfying the preference encoded by $g$. The regularized minimum can move relative to the unregularized loss surface.

== Probabilistic Interpretation

Maximum likelihood chooses parameters that maximize:

$ product_(i=1)^I Pr(y_i | x_i, phi) $

Regularization can be interpreted as a prior over parameters:

$ hat(phi) = argmax_(phi) [product_(i=1)^I Pr(y_i | x_i, phi) Pr(phi)] $

This is maximum a posteriori estimation. Taking negative logs gives:

$ lambda g(phi) = - log Pr(phi) $

so the regularizer is the negative log prior up to scaling.

== L2 Regularization

The most common explicit regularizer is the squared $L_2$ norm:

$ g(phi) = sum_j phi_j^2 $

The objective becomes:

$ hat(phi) = argmin_(phi) [sum_(i=1)^I ell_i(x_i, y_i) + lambda sum_j phi_j^2] $

For neural networks, this penalty is usually applied to weights rather than biases. It is often called weight decay, though the notes point out that strict weight decay and L2 regularization are not identical in all optimizer settings.

L2 regularization encourages small weights, which tends to make the represented function smoother. If all weights were forced to zero, the network would reduce to a constant output determined by the final bias.

Small $lambda$ has little effect. Intermediate $lambda$ can reduce overfitting by smoothing the learned function. Large $lambda$ can underfit because the regularization term overwhelms the data term.

== Implicit Regularization

Gradient descent and SGD do not choose neutrally among all low-loss solutions. Their trajectories are biased by step size, batch size, initialization, and update noise. This is implicit regularization.

For continuous gradient flow:

$ (d phi) / (d t) = - (partial L) / (partial phi) $

Discrete gradient descent approximates this with finite steps:

$ phi_(t+1) = phi_t - alpha (partial L(phi_t)) / (partial phi) $

The finite step size changes the path. One approximation is that discrete gradient descent behaves like continuous descent on a modified loss:

$ tilde(L)_("GD")(phi) = L(phi) + alpha / 4 norm((partial L) / (partial phi))^2 $

This penalizes regions where the gradient norm is large, changing the optimization trajectory even when the original minima are unchanged.

For SGD, there is an additional term related to the variance of batch gradients. SGD implicitly favors places where different batches agree on the slope. Smaller batches and larger learning rates can therefore influence generalization, not just speed.

== Early Stopping

Early stopping halts training before convergence. It helps when the model first captures coarse structure and later overfits to noise.

It can be understood in two ways:

- weights initialized small do not have enough time to grow large, similar to L2 regularization
- stopping early reduces effective model complexity, moving back along the bias-variance tradeoff

The stopping time is selected using validation performance. Unlike many hyperparameters, it can be chosen from a single training run by saving checkpoints and keeping the parameters with the best validation result.

== Ensembling

An ensemble averages predictions from multiple models. For regression, outputs can be averaged or medians can be used. For classification, one can average logits or probabilities, or vote over predicted classes.

Ensembling reduces variance when different models make partially independent errors. Models can differ by random initialization, training data resampling, hyperparameters, or model families.

Bootstrap aggregating, or bagging, trains models on datasets sampled with replacement. Outliers may be absent from some resampled datasets, so averaging can produce a smoother and more robust predictor.

The cost is training, storing, and running multiple models.

== Dropout

Dropout randomly clamps a subset of hidden units to zero during each SGD iteration. This trains a different subnetwork at each step while sharing parameters across subnetworks.

Dropout discourages hidden units from relying on fragile co-adapted partners. It also removes unnecessary kinks or local changes in regions that do not affect training loss but may generalize poorly.

At inference, a common approximation uses all hidden units and scales weights by one minus the dropout probability. Monte Carlo dropout instead runs the network multiple times with dropout active and combines the predictions, approximating an ensemble.

== Applying Noise

Dropout is multiplicative Bernoulli noise on activations. The same principle extends to other noise sources:

- input noise smooths the learned function and can be related to penalizing input derivatives
- adversarial training uses worst-case small input perturbations
- weight noise favors wide, flat minima where small parameter changes matter less
- label smoothing discourages overconfident classification by replacing one-hot labels with softened target distributions

Noise can improve generalization because the model must be stable under perturbations rather than merely fit exact observed examples.

== Bayesian Inference

Maximum likelihood picks one parameter vector. Bayesian inference treats parameters as uncertain and computes a posterior:

$ Pr(phi | {x_i, y_i}) = (product_(i=1)^I Pr(y_i | x_i, phi) Pr(phi)) / (integral product_(i=1)^I Pr(y_i | x_i, phi) Pr(phi) dif phi) $

Prediction averages over parameter uncertainty:

$ Pr(y | x, {x_i, y_i}) = integral Pr(y | x, phi) Pr(phi | {x_i, y_i}) dif phi $

This is an infinite weighted ensemble and can express predictive uncertainty. For large neural networks, the full posterior and integral are intractable, so practical Bayesian deep learning relies on approximations.

== Transfer, Multi-Task, and Self-Supervised Learning

Transfer learning pre-trains a model on a related data-rich task, replaces or adapts the final layers, and then trains or fine-tunes on the target task. The pretrained network supplies a useful representation and a good initialization.

Multi-task learning trains one network on several tasks at once. Shared representations can improve each task when the tasks require related understanding.

Self-supervised learning creates labels from the data itself. Generative methods mask part of an input and train the model to reconstruct it. Contrastive methods train the model to identify related versus unrelated pairs, such as two augmented views of the same image.

== Data Augmentation

Data augmentation expands the dataset by applying transformations that preserve the label. For images this may include flips, rotations, color changes, blur, or crops. For text it may include synonym substitution or back-translation. For audio it may alter frequency bands or volume.

The key constraint is label preservation. If the transformation changes the correct target, augmentation injects label noise rather than useful invariance.

== Summary Principles

The methods in the chapter improve generalization through four broad mechanisms:

- encourage smoother functions
- increase the effective amount of data
- combine multiple models
- encourage wide minima where parameter errors matter less

Architecture choice can also regularize. Convolutional networks, graph neural networks, and transformers build task structure directly into the model family.

== Takeaway

Regularization selects among solutions with similar training behavior by preferring smoother, more stable, better-supported, or better-averaged predictors that generalize more reliably.
