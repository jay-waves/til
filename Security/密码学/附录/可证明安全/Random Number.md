## Pseudorandom Generator

Let $G$ be a deterministic polynomial-time algorithm such that for any $n$ and any input $s \in \{0,1\}^n$, the result $G(s)$ is a string of length $\ell(n)$. $G$ is a pseudorandom generator if the following conditions hold:
1. **(Expansion).** For every $n$ it holds that $\ell(n) > n$.
2. **(Pseudorandomness).** For any PPT algorithm $D$, there is a negligible function $\text{negl}$ such that $$ | \Pr[D(G(s)) = 1] - \Pr[D(r) = 1] | \leq \text{negl}(n), $$ where the first probability is taken over uniform choice of $s \in \{0,1\}^n$ and the randomness of $D$, and the second probability is taken over uniform choice of $r \in \{0,1\}^{\ell(n)}$ and the randomness of $D$.

We call $\ell(n)$ the expansion factor of $G$.

## Pseudorandom Function

An efficient, length preserving, keyed function $F : \{0,1\}^* \times \{0,1\}^* \rightarrow \{0,1\}^*$ is a pseudorandom function if for all PPT distinguishers $D$, there is a negligible function $\text{negl}$ such that: 
$$\left| \Pr[D^{F_k(\cdot)}(1^n) = 1] - \Pr[D^{f(\cdot)}(1^n) = 1] \right| \leq \text{negl}(n)$$
where the first probability is taken over uniform choice of $k \in \{0,1\}^n$ and the randomness of $D$, and the second probability is taken over uniform choice of $f \in \text{Func}$, and the randomness of $D$.

$$G(s)\stackrel{def}{=}F_{s}(1)\Vert F_{s}(2)\Vert \dots F_{s}(\ell)$$