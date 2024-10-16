## Private Key Enc

### Indistinguishability Exp

**The CPA indistinguishability experiment** $\text{PrivK}^{\text{cpa}}_{A,\Pi}(n)$:
1. A key $k$ is generated by running $\text{Gen}(1^n)$.
2. The adversary $A$ is given input $1^n$ and oracle access to $\text{Enc}_k(\cdot)$, and outputs a pair of messages $m_0, m_1$ of the same length.
3. A uniform bit $b \in \{0, 1\}$ is chosen, and then a ciphertext $c \leftarrow \text{Enc}_k(m_b)$ is computed and given to $A$.
4. The adversary $A$ continues to have oracle access to $\text{Enc}_k(\cdot)$, and outputs a bit $b'$.
5. $\text{PrivK}^{\text{cpa}}_{A,\Pi}(n)=1$ if $b' = b$, and $0$ otherwise. In the former case, we say that $A$ succeeds.

$\Pi = (\text{Gen}, \text{Enc}, \text{Dec})$ is CPA-secure if for all PPT adversaries $A$, there is a negligible function $\text{negl}$ such that:
$$ \Pr[\text{PrivK}^{\text{cpa}}_{A,\Pi}(n) = 1] \leq \text{negl}(n). $$

### LR-oracle CPA experiment

We now give $\mathcal{A}$ access to a `left-or-right` oracle $LR_{k,b}$ that, on input a pair of equal-length messages $m_{i,0}, m_{i,1}$, computes $c \leftarrow \text{Enc}_{k}(m_{i,b})$ and returns $c$. That is, on multiple pair of messages $(m_{00},m_{01}),(m_{10},m_{11}),\dots,(m_{i0},m_{i1}),\dots$, if $b = 0$ then the adversary always receives an encryption of the `left` plaintext, and if $b = 1$ then it always receives an encryption of the `right` plaintext. 

![|450](../../../../attach/Pasted%20image%2020231228175459.png)

**The LR-oracle experiment** $\text{PrivK}^{LR-cpa}_{A,\Pi}(n)$:
1. A key $k$ is generated by running $\text{Gen}(1^n)$.
2. A uniform bit $b \in \{0, 1\}$ is chosen.
3. The adversary $A$ is given input $1^n$ and oracle access to $LR_{k,b}(\cdot,\cdot)$, as defined above.
4. The adversary $A$ outputs a bit $b'$.
5. The output of the experiment is defined to be $1$ if $b' = b$, and $0$ otherwise. In the former case, we say that $A$ succeeds.

$\Pi$ has indistinguishable encryptions under CPA for multiple encryptions, *if* for all PPT adversaries $\mathcal{A}$ there is a negligible function $\text{negl}$ such that
$$\Pr[\text{PrivK}^{LR-cpa}_{A,\Pi}(n) = 1] \leq \frac{1}{2} + \text{negl}(n)$$

### CPA-secure scheme from PRNF

Let $F$ be a pseudorandom function. Define a fixed-length, private-key encryption scheme for messages of length $n$ as follows: 
- Gen: on input $1^{n}$, choose uniform $k\in\set{0,1}^{n}$ and output it. 
- Enc: on input a key $k\in\set{0,1}^{n}$ and a message $m\in\set{0,1}^{n}$, choose uniform $r\in\set{0,1}^{n}$ and output the ciphertext $$c := \langle r, F_k(r) \oplus m \rangle$$
- Dec: on input a key $k\in\set{0,1}^{n}$ and a ciphertext $c=\langle r,s\rangle$, output the message $$m:=F_{k}(r)\oplus s$$

## Public Key Enc

*If a public-key encryption scheme has indistinguishable encryptions in the presence of an eavesdropper, it is CPA-secure.*[^1]

### LR-oracle CPA experiment

Formally, consider the following experiment defined for an adversary $A$ and a public-key encryption scheme $\Pi = (\text{Gen}, \text{Enc}, \text{Dec})$:

**The LR-oracle experiment** $\text{PubK}^{\text{LR-cpa}}_{A,\Pi}(n)$:
1. $\text{Gen}(1^n)$ is run to obtain keys $(pk, sk)$.
2. A uniform bit $b \in \{0,1\}$ is chosen.
3. The adversary $\mathcal{A}$ is given input $pk$ and oracle access to $\text{LR}_{pk, b}(\cdot, \cdot)$.
4. The adversary $\mathcal{A}$ outputs a bit $b'$.
5. $\text{PubK}^{\text{LR-cpa}}_{A,\Pi}(n) = 1$ if $b' = b$, and $0$ otherwise.

A PubK Scheme $\Pi$ has **indistinguishable multiple encryptions** if for all PPT adversaries $\mathcal{A}$ there exists a $\mathsf{negl}$ such that: $$Pr[\mathsf{PubK}^{LR-cpa}_{\mathcal{A},\Pi}(n)=1]\leq \frac{1}{2}+\mathsf{negl}(n)$$

[^1]: P406, Proposition 12.3, *Introduction to Modern Cryptography* by Jonathan Katz