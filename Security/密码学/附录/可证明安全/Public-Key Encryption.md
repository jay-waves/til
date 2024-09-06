## Definition[^definition12.1] 

A public-key encryption scheme (PKE) is a triple of probabilistic polynomial-time algorithms $(\text{Gen}, \text{Enc}, \text{Dec})$ such that:

1. The key-generation algorithm Gen takes as input the security parameter $1^n$ and outputs a pair of keys $(pk, sk)$. We assume for convenience that $pk$ and $sk$ each has length at least $n$, and that $n$ can be determined from $pk, sk$.
2. The public key $pk$ defines a message space $M_{pk}$.
3. The encryption algorithm Enc takes as input a public key $pk$ and message $m \in M_{pk}$, and outputs a ciphertext $c$; we denote this by $c \leftarrow \text{Enc}_{pk}(m)$. (Enc will need to be probabilistic to achieve meaningful security.)
4. The deterministic decryption algorithm Dec takes as input a private key $sk$ and a ciphertext $c$, and outputs a message $m$ or a special symbol $\bot$ denoting failure. We write this as $m := \text{Dec}_{sk}(c)$.

It is required that, except with negligible probability over the randomness of Gen and Enc, we have $\text{Dec}_{sk}(\text{Enc}_{pk}(m)) = m$ for any message $m \in M_{pk}$.

### Security

Given a public-key encryption scheme $\Pi = (\text{Gen}, \text{Enc}, \text{Dec})$ and an adversary $A$, consider the following experiment:

**The eavesdropping indistinguishability experiment** $\text{PubK}^{\text{eav}}_{A,\Pi}(n)$:
1. $\text{Gen}(1^n)$ is run to obtain keys $(pk, sk)$.
2. Adversary $A$ is given $pk$, and outputs a pair of equal-length messages $m_0, m_1 \in M_{pk}$.
3. A uniform bit $b \in \{0,1\}$ is chosen, and then a ciphertext $c \leftarrow \text{Enc}_{pk}(m_b)$ is computed and given to $A$. We call $c$ the challenge ciphertext.
4. $A$ outputs a bit $b'$. $\text{PubK}^{\text{eav}}_{A,\Pi}(n)=1$ if $b' = b$, and $0$ otherwise.

A public-key encryption scheme $\Pi = (\text{Gen}, \text{Enc}, \text{Dec})$ has indistinguishable encryptions in the presence of an eavesdropper if for all PPT adversaries $A$ there is $\text{negl}$ such that
$$\Pr[\text{PubK}^{\text{eav}}_{A,\Pi}(n) = 1] \leq \frac{1}{2} + \mathsf{negl}(n)$$

### Theorem 12.6[^2]

*If public-key encryption scheme $\Pi$ is CPA-secure, then it also has indistinguishable $\mathsf{multiple}$ encryptions*.

## Hybrid encryption

Let $\Pi = (\text{Gen}, \text{Encaps}, \text{Decaps})$ be a KEM with key length $n$, and let $\Pi' = (\text{Gen}', \text{Enc}', \text{Dec}')$ be a private-key encryption scheme. Construct a public-key encryption scheme $\Pi^{hy} = (\text{Gen}^{hy}, \text{Enc}^{hy}, \text{Dec}^{hy})$ as follows:

- $\text{Gen}^{hy}$: on input $1^n$ run $\text{Gen}(1^n)$ and use the public and private keys $(pk, sk)$ that are output.
- $\text{Enc}^{hy}$: on input a public key $pk$ and a message $m \in \{0,1\}^*$ do:
	1. Compute $(c, k) \leftarrow \text{Encaps}_{pk}(1^n)$.
	2. Compute $c' \leftarrow \text{Enc}'_k(m)$.
	3. Output the ciphertext $(c, c')$.
- $\text{Dec}^{hy}$: on input a private key $sk$ and a ciphertext $(c, c')$ do:
	1. Compute $k := \text{Decaps}_{sk}(c)$.
	2. Output the message $m := \text{Dec}'_k(c')$.

![|300](../../../../attach/Pasted%20image%2020231228193014.png)

### Security of Hybrid Enc

*If $\Pi$ is a CPA-secure KEM, and $\Pi '$ is an [EAV-secure](Private-Key%20Encryption.md) private-key enctryption scheme, then $\Pi^{hy}$ is a [CPA-secure](CPA-Secure.md) public-key encryption scheme.*[^3]

### KEM

A **key-encapsulation mechanism (KEM)** is a tuple of PPT algorithms $(\text{Gen}, \text{Encaps}, \text{Decaps})$ such that:
1. $(pk,sk)\leftarrow \mathsf{Gen}(1^n)$. Gen takes as input the security parameter $1^n$ and outputs a public-/private-key pair $(pk, sk)$. We assume $pk$ and $sk$ each has length at least $n$, and that $n$ can be determined from $pk$.
2. $(c, k) \leftarrow \text{Encaps}_{pk}(1^n)$. The encapsulation algorithm Encaps takes as input a public key $pk$ (which implicitly defines $n$). It outputs a ciphertext $c$ and a key $k \in \{0, 1\}^{\ell(n)}$ where $\ell$ is the key length.  
3. $k/\bot := \text{Decaps}_{sk}(c)$. The deterministic decapsulation algorithm Decaps takes as input a private key $sk$ and a ciphertext $c$, and outputs a key $k$ or a special symbol $\bot$ denoting failure.

It is required that with all but negligible probability over the randomness of Gen and Encaps, if $\text{Encaps}_{pk}(1^n)$ outputs $(c, k)$ then $\text{Decaps}_{sk}(c)$ outputs $k$.

$KEM^{CPA}_{\mathcal{A},\Pi}$: 
![450](../../../../attach/Pasted%20image%2020231228193315.png)

## Glog-Based PKE

### ElGamal Enc

Let $\mathcal{G}$ be as a polynomial-time algorithm that takes as input $1^n$ and outputs a description of a cyclic group $\mathbb{G}$, its order q ($\Vert q\Vert =n$), and a generator $g$. Define a PKE scheme as follows:

- Gen: on input $1^n$ run $\mathcal{G}(1^{n})$ to obtain $(\mathbb{G},q,g)$. Then choose a uniform $x \in \mathbb{Z}_q$ and compute $h := g^x$. The public key is $(G, q, g, h)$ and the private key is $(G, q, g, x)$. The message space is $\mathbb{G}$.
- Enc: on input a public key $pk = (\mathbb{G}, q, g, h)$ and a message $m\in \mathbb{G}$, choose a uniform $y \in \mathbb{Z}_q$ and output the ciphertext $$(g^y, h^y \cdot m)$$
- Dec: on input a private key $sk = (\mathbb{G}, q, g, x)$ and a ciphertext $(c_1, c_2)$, output $$\hat{m} := c_2/c_1^x$$

#### Security of ElGamal

*If the DDH problem is hard relative to $\mathbb{G}$, then the El Gamal encryption scheme is CPA-scure* [^4]

ElGamal is not secure against CCA (is malleable)

## RSA-Based PKE

> see [RSA](../../公钥密码/RSA/RSA.md)

[^definition12.1]: P404, *Introduction to Modern Cryptography* by Jonathan Katz
[^2]: P408
[^3]: P420
[^4]: P428