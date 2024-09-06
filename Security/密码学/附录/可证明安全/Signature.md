## security of signature

The formal definition of security is essentially the same as Definition 4.2, with the main difference being that here the adversary is given a public key. Let $\Pi = (\text{Gen}, \text{Sign}, \text{Vrfy})$ be a signature scheme, and consider the following experiment for an adversary $\mathcal{A}$ and parameter $n$:

**The signature experiment** $\text{Sig-forge}_{A,\Pi}(n)$:
1.  $\text{Gen}(1^n)$ is run to obtain keys $(pk,sk)$.
2. Adversary $\mathcal{A}$ is given $pk$ and access to an oracle $\text{Sign}_{sk}(\cdot)$. The adversary then outputs $(m, \sigma)$. Let $\mathcal{Q}$ denote the set of all queries that $\mathcal{A}$ asked its oracle.
3. $\mathcal{A}$ succeeds if and only if (1) $\text{Vrfy}_{pk}(m, \sigma) = 1$ and (2) $m\notin \mathcal{Q}$. In this case the output of the experiment is defined to be 1.

A signature scheme $\Pi$ is *existentially unforgeable under an adaptive chosen-message attack*, or just secure, if for all probabilistic polynomial-time adversaries $\mathcal{A}$, there is a negligible function $\mathsf{negl}$ such that: $$\Pr[\mathsf{Sig-forge}_{A,\Pi}(n) = 1] \leq \text{negl}(n)$$

## RSA-based Signature

### RSA-FDH

Let GenRSA be as in the previous sections, and construct a signature scheme as follows:

- **Gen**: on input $1^{n}$, run $GenRSA(1^n)$ to compute $(N, e, d)$. The public key is $(N, e)$ and the private key is $(N, d)$.  
  As part of key generation, a function $H : \set{0,1}^* -> \mathbb{Z}_N^*$ is specified as a **Full Domain Hash** which is a random oracle that maps its inputs uniformly onto $\mathbb{Z}^{*}_{N}$
- **Sign**: on input a private key $(N, d)$ and a message $m$ in $\{0,1\}*$, compute
  $$σ := [H(m)^d \pmod{N}]$$

- **Vrfy**: on input a public key $(N, e)$, a message $m$, and a signature $\sigma$, output 1 if and only if
  $$σ^e\stackrel{?}{=}H(m) \pmod{N}$$

*If the RSA problem is hard relatie to $\mathsf{GenRSA}$ and $H$ is modeled as a random oracle, then RSA-FDH is secure.*