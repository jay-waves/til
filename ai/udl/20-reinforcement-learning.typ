#import "../../appx/theme.typ": tufte, meta, note

#show: tufte

#meta(
  subtitle: [Book chapter 19],
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

= PDF 20: Reinforcement Learning

== Motivation

Reinforcement learning differs from supervised learning because the learner's actions influence the data it observes. An agent interacts with an environment, receives rewards, and must choose actions that maximize long-term return.

== Markov Decision Process

An MDP has states $s$, actions $a$, transition distribution, reward distribution, and discount $gamma$.

Markov property:

$ Pr(s_(t+1) | s_t, a_t, s_(t-1), dots) = Pr(s_(t+1) | s_t, a_t) $

The current state is assumed to contain all information needed to predict future transitions given the action. If observations are not Markov, the agent may need memory or state estimation.

== Policy and Return

A stochastic policy is:

$ a_t tilde pi(a | s_t) $

Discounted return:

$ G_t = sum_(k=0)^infinity gamma^k r_(t + k + 1) $

The discount $gamma in [0, 1]$ controls how much future rewards matter. Smaller values make the agent more short-sighted.

== Value Functions

$ v_("pi")(s) = E_("pi") [G_t | s_t = s] $

$ q_("pi")(s, a) = E_("pi") [G_t | s_t = s, a_t = a] $

The state-value function evaluates how good it is to be in a state under policy $pi$. The action-value function evaluates taking action $a$ first and following $pi$ afterward.

== Bellman Equations

$ v_("pi")(s) = sum_a pi(a | s) sum_(s', r) Pr(s', r | s, a) (r + gamma v_("pi")(s')) $

$ q_("pi")(s, a) = sum_(s', r) Pr(s', r | s, a) (r + gamma sum_(a') pi(a' | s') q_("pi")(s', a')) $

These equations express current value as immediate reward plus discounted future value.

The Bellman optimality equation replaces the policy-weighted future action with the best future action. It is the basis for control algorithms that improve behavior.

== Dynamic Programming

When the transition and reward distributions are known and the state-action space is small, dynamic programming can solve tabular reinforcement learning problems.

Policy evaluation repeatedly applies Bellman updates to estimate values for a fixed policy. Policy improvement updates the policy greedily with respect to the estimated action values. Policy iteration alternates these steps until the policy stops changing.

Value iteration combines evaluation and improvement by directly applying Bellman optimality-style updates.

== Monte Carlo and Temporal Difference Learning

Monte Carlo methods estimate returns from completed trajectories. They are unbiased with respect to sampled episodes but can have high variance and require waiting for episode completion.

Temporal-difference methods bootstrap from current value estimates:

$ "target" = r_(t+1) + gamma v(s_(t+1)) $

Bootstrapping improves sample efficiency but can propagate estimation bias.

== Q-Learning

$ q(s_t, a_t) arrow q(s_t, a_t) + alpha (r_(t+1) + gamma max_a q(s_(t+1), a) - q(s_t, a_t)) $

The parenthesized term is the temporal-difference error. Bootstrapping improves sample efficiency but can propagate bias.

Q-learning is off-policy: it can learn the greedy action-value function while collecting data with an exploratory behavior policy.

== Exploration

Exploration is essential because a policy that only exploits current value estimates may never discover better actions. Simple methods include epsilon-greedy behavior; more advanced methods use uncertainty, entropy bonuses, or intrinsic rewards.

== Deep Reinforcement Learning

Deep Q-networks approximate $q(s, a)$ with neural networks. They add stabilizers:

- replay buffers store transitions and sample them later, reducing temporal correlation
- target networks provide a slowly changing bootstrap target
- clipping, normalization, and careful exploration improve stability

Policy-gradient methods optimize a parameterized policy directly. Actor-critic methods combine an actor policy with a critic value function; the critic provides lower-variance learning signals for the actor.

== Policy Gradients and Actor-Critic

Policy gradient methods directly optimize the expected return of a stochastic policy $pi_theta(a | s)$. They estimate how changing policy parameters changes the probability of sampled trajectories and therefore expected reward.

REINFORCE is a Monte Carlo policy-gradient method. It is general but high variance. Baselines reduce variance without changing the expected gradient by subtracting a value estimate from the return.

Actor-critic methods combine policy gradients with temporal-difference learning. The actor updates the policy; the critic estimates state values or action values and supplies lower-variance learning signals such as advantages.

== Offline RL

Offline RL learns from a fixed dataset rather than collecting new experience. This is attractive when environment interaction is expensive or risky.

The central difficulty is distribution shift. A learned policy may choose actions that are rare or absent in the dataset, where value estimates are extrapolations. Offline methods therefore constrain the policy or penalize unsupported actions.

== Notes

Policy evaluation estimates the value of a fixed policy. Control improves the policy. Many algorithms alternate between these two problems.

RL is hard because data are sequential, non-i.i.d., policy-dependent, and often sparse in reward. Neural networks add approximation error and instability to the already difficult credit-assignment problem.

== Takeaway

Reinforcement learning optimizes long-term reward in policy-dependent data. MDPs formalize the problem, Bellman equations relate present and future value, and deep RL uses neural approximators with stabilizers to learn value functions or policies.
