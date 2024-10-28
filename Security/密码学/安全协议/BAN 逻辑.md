BAN Logic, Burrows, Abadi and Needham, 1989. 用于对认证协议形式化分析.

基本假设:
1. 时间假设:
	1. 时间段分为: 过去和现在
	2. *现在*开始于协议开始阶段, 此前皆是*过去*
	3. 某个*信仰*在协议开始时建立, 那么在整个*现在*时间段都成立. *过去*的信仰则不一定在*现在*成立.
2. 密钥假设: 
	1. 加密算法是完美保密的
	2. 密文块无法被篡改
	3. 密文包含足够冗余信息, 解密者可以通过解密结果判断其是否解密成功
	4. 消息中有足够冗余信息, 主体可以判断该消息是否来源于自己
3. 主体假设: 假设协议参与的主体是诚实的

分析步骤:
- 理想化协议流程
- 确认初始假设
- 逻辑推理
- 得出结论

缺点:
- 初始假设及理想化过程过于依赖分析者直觉, 没有标准转换格式.
- **缺乏完备性**, 可以分析出漏洞; 但是被BAN逻辑证明是安全的, 却不一定是安全的.
- 缺乏语义.

### Signal

- Principals, 主体: $A, B$
- Shared Key: $K_{AB}$
- Public Key: $K_A$
- Private Key: $K_A^{-1}$
- Nonce, 随机数: $N_A$
- Formula, Statements: $X, Y$

Basic Formula:
- Encryption: $\{X\}_{K}$
- Compose: $<X>_{Y}$, X与秘密Y合成的消息
- Believe: $A\ \lvert\equiv X$
- See: $A\ \triangle\ X$, A曾收到包含X的消息
- Said: $A\ \lvert\sim X$, 消息源是A, 并且A能够确认X的含义
- Freshness: $\#X$, 协议执行前未被传输过
- Control: $A\ \lvert \Rightarrow  X$, 有管辖权
- Share Key: $A\stackrel{K}{\longleftrightarrow}B$, K仍有保密性
- Share Statement: $A \stackrel{X}\Longleftrightarrow B$, A和B分享秘密X
- Public Key: $\stackrel{K}{\longrightarrow} A$, K是A公开的公钥

### Rule

#### 1 消息含义规则 (Message Meaning Rule)

消息含义规则使主体推知消息的出处.

对称密钥: R1 
$$\frac{P\lvert\equiv P\stackrel{K}{\longleftrightarrow} Q,\ P\triangle \{X\}_{K}}{P\lvert\equiv Q\lvert\sim X}$$

公钥: R2
$$\frac{P\lvert\equiv \stackrel{K_{Q}}{\longrightarrow} Q,\ P\triangle \{X\}_{K^{-1}}}{P\lvert\equiv Q\lvert\sim X}$$

共享秘密: R3 ==?==
$$\frac{P\lvert\equiv P\stackrel{Y}{\Longleftrightarrow} Q,\ P\triangle <X>_{Y}}{P\lvert\equiv Q\lvert\sim X}$$

#### 2 随机数验证规则 (Nonce-Verification Rule)

P相信X是新鲜的, 且P相信Q曾经说过X, 那么P相信Q相信X是真实的. R4

$$\frac{P\ \lvert\equiv \#(X),\ P\ \lvert\equiv Q\ \lvert\sim X}{P\ \lvert\equiv Q\ \lvert\equiv X}$$

#### 3 接收消息规则 (Seeing Rules)
R6:
$$\frac{P\ \triangle\ (X,\ Y)}{P\ \triangle\ X}$$
R7:
$$\frac{P\ \triangle\ <X>_{Y}}{P\ \triangle\ X}$$

对称密钥: R8
$$\frac{P\lvert\equiv Q\stackrel{K_{PQ}}{\longleftrightarrow}P,\ P\ \triangle\ \{X\}_{K_{PQ}}}{P\ \triangle\ X}$$

公钥加密 (解密): R9  
$$\frac{P\ \lvert\equiv \stackrel{K}{\longrightarrow}P,\ P\ \triangle\ \{X\}_{K}}{P\ \triangle\ X}$$

签名: R10
$$\frac{P\ \lvert\equiv \stackrel{K}{\longrightarrow}Q,\ P\ \triangle\ \{X\}_{K^{-1}}}{P\ \triangle\ X}$$

#### 4 新鲜性规则

主体P相信消息的一部分X是新鲜的, 那么也相信消息整体式新鲜的. R11

$$\frac{P\ \lvert\equiv\ \#(X)}{P\ \lvert\equiv\ \#(X, Y)}$$

#### 5 信仰规则 (Belief Rule)
R12:
$$\frac{P\lvert\equiv X,\ P\lvert\equiv Y}{P\lvert\equiv (X,\ Y)}$$
R13:
$$\frac{P\lvert\equiv(X,\ Y)}{P\lvert\equiv X}$$

==为什么这条能作为基本原则? 不是第二条推的么?==
R14:
$$\frac{P\lvert\equiv Q\lvert\equiv (X,\ Y)}{P\lvert\equiv Q\lvert\equiv X}$$
R15:
$$\frac{P\lvert\equiv Q\lvert \sim(X,\ Y)}{P\lvert\equiv Q\lvert \sim X}$$

#### 6 密钥与秘密共享规则 (Shared Key and Secret Rules)

共享密钥和共享秘密都有对称性, 即共享是没有方向的.

R16:
$$\frac{P\lvert\equiv R\stackrel{X}{\longleftrightarrow}R'}{P\lvert\equiv R'\stackrel{X}{\longleftrightarrow}R}$$
R17:
$$\frac{P\lvert\equiv Q\lvert\equiv R\stackrel{X}{\longleftrightarrow}R'}{P\lvert\equiv Q\lvert\equiv R'\stackrel{X}{\longleftrightarrow}R}$$
R18:
$$\frac{P\lvert\equiv R\stackrel{X}{\Longleftrightarrow}R'}{P\lvert\equiv R'\stackrel{X}{\Longleftrightarrow}R}$$
R19:
$$\frac{P\lvert\equiv Q\lvert\equiv R'\stackrel{X}{\longleftrightarrow}R}{P\lvert\equiv Q\lvert\equiv R\stackrel{X}{\longleftrightarrow}R'}$$

#### 7 管辖规则 (Jurisdiction Rule)

拓展了主体的推知能力. ==?== R5

$$\frac{P\lvert\equiv Q\lvert\Rightarrow X,\ P\lvert\equiv Q\lvert\equiv X}{P\lvert\equiv X}$$

### 理想化协议流程

**替换共享密钥**: 在 BAN 逻辑中, 不直接使用原始的共享密钥. 相反, 会使用一个表示 "实体 A 和实体 B 共享密钥 K" 的陈述来替换它.

**使用公钥**: 使用表示 "实体 A 的公钥是 K" 的陈述来替换公钥.

**将协议一条消息抽象为 see**, 比如 A 发送给 B 一条消息, 只能抽象为 A see, 而不能抽象为 B said.