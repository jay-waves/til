KMAC 使用 SHA-3 哈希函数构造 MAC, 由于 SHA-3 使用了 Keccak 结构, 不存在*长度延长攻击*, 所以不需要类似 HMAC 的嵌套结构.

$KMAC=Keccak(k\ \Vert\ x)$

### 构造中的一个问题

**为什么构造 $Keccak(x\Vert k)$ 弱于 $Keccak(k\Vert x)$**?

类似 [长度延长攻击](Security/密码学/消息摘要/MD%20迭代结构/长度延长攻击.md) 思路, 敌手在 $Keccak(x\Vert k)$ 海绵结构的 $x$ 部分进行碰撞攻击, 找到 $x'$ 生成相同的状态 (state) 进入 $k$ 部分:

> 论文 “[Cryptographic sponge functions](https://keccak.team/files/CSF-0.1.pdf)” 章节 5.11.2 提到:   
> Note that one can also define a MAC function by taking as input the message followed by the key: $t = \left\lfloor {F(M \mathbin\Vert K)} \right\rfloor _n$$t = \left\lfloor {F(M \mathbin\Vert K)} \right\rfloor _n$. In this case, an adversary has the advantage that she can try to generate inner collisions *offline*, i.e., without having to query the keyed sponge function. Additionally, she can try to construct a path to a state that occurs in the absorbing of a target message, leading to a second message with the same MAC.

## 参考

> [Security difference between $Keccak(k\Vert x)$ and $Keccak(x\Vert k)$ - Stack Exchange](https://crypto.stackexchange.com/questions/61154/security-difference-between-mathrmkeccakk-mathbin-x-and-mathrmkeccak?noredirect=1&lq=1)  
> KMAC 首次定义于 [FIPS 800-185](https://nvlpubs.nist.gov/nistpubs/SpecialPublications/NIST.SP.800-185.pdf)
