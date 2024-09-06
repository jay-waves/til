## Perfect Secrecy

- adversary knows the probability distribution of M
- adversary knows the encryption scheme
- adversary can *eavesdrop* the ciphertext (ciphertext-only attack)

Perfect Secrecy is that **the [posteriori probability](../../../../Math/概率论与随机过程/贝叶斯公式.md) that some message $m\in M$ was sent, conditioned on the ciphertext that was observed, should be no different from the a [priori probability](../../../../Math/概率论与随机过程/贝叶斯公式.md#贝叶斯定理) that $m$ would be sent.**

Formally, 

#### Definition 2.3 [^2] 

*An encryption scheme $(Gen, Enc, Dec)$ with message space $M$ is perfectly secret if:*

*for every probability distribution for $M$, every message $m\in M$, and every ciphertext $c\in C$ for which $Pr[C = c] > 0$ (not-zero probability):* $$Pr[\mathbf{M} = m\ \vert\ \mathbf{C} = c] = Pr[\mathbf{M} = m]$$

**Equivalent Definition:**

$Pr[\mathbf{C}=c\vert \mathbf{M}=m]=Pr[\mathbf{C}=c]$

#### Lemma 2.5 [^3]

*An encryption scheme (Gen, Enc, Dec) with message space $\mathcal{M}$ is perfectly secret if and only if $Pr[\mathbf{Enc}_{K}(m)=c]=Pr[\mathbf{Enc}_{K}(m')=c]$ holds for every $m,\ m'\in\mathcal{M}$ and every $c\in \mathcal{C}$.*

$Pr[\mathbf{Enc}_{K}(m)=c]\equiv Pr[\mathbf{C}=c\vert \mathbf{M}=m]$

## One-Time Pad

Fix an integer $\lambda> 0$. The message space $\mathcal{M}$, key space $\mathcal{K}$, and ciphertext space $\mathcal{C}$ are all equal to $\{0, 1\}^\lambda$

- **Gen**: the key-generation algorithm chooses a key from $\mathcal{K}=\{0,1\}^\lambda$ according to the uniform distribution. (i.e. each of the $2^{\lambda}$ strings in the space is chosen as the key with probability exactly $2^{-\lambda}$ )

- **Enc**: given a key $k\in\mathcal{K}=\{0,1\}^\lambda$ and a message $m\in\{0,1\}^\lambda$, the encryption algorithm outputs the ciphertext $c:=k\oplus m$.

- **Dec**: given a key $k\in\mathcal{K}=\{0,1\}^\lambda$ and a ciphertext $c\in\{0,1\}^\lambda$ , the decryption algorithm outputs the message $m:=k\oplus c$

#### Theorem 2.11 [^1]

*If (Gen, Enc, Dec) is a  prefectly secret encryption scheme with message spcae $\mathcal{M}$ and key space $\mathcal{K}$, then $\vert \mathcal{K}\vert\ge \vert \mathcal{M}\vert$.*

**Proof:**

Assume $\vert \mathcal{K}\vert<\vert \mathcal{M}\vert$, Let $\mathcal{M}(c)$ be the set of all possible messages that are decryptions of c. that is $$\mathcal{M}(c)\overset{def}{=}\{m\ \vert\ m=\mathbf{Dev}_{k}(c)\text{ for some }k\in\mathcal{K}\}$$ Clearly $\vert\mathcal{M}(c)\vert\ \leq\ \vert\mathcal{K}\vert$ (**Dec** is deterministic). If  $\vert \mathcal{K}\vert<\vert \mathcal{M}\vert$, there is some $m'\in\mathcal{M}$ such that $m'\not\in \mathcal{M}(c)$. But then:$$Pr[M=m'\vert C=c]=0\neq Pr[M= m']$$

## Perfect Indistinguishable

> see experiment in [eav-secure](Private-Key%20Encryption.md)

![|450](../../../../attach/Pasted%20image%2020231229112027.png)

$\Pi$ is perfectly indistinguishable if fro all attackers $\mathcal{A}$, holds that: $$Pr[\mathsf{PrivK}^{eav}_{\mathcal{A},\Pi} =1]=\frac{1}{2}$$

**Perfectly Indistinguishable $\equiv$ Perfect Secret**

[^1]: P34
[^2]: P27 in *Introduction to Modern Cryptography* by Jonathan Katz
[^3]: P28
