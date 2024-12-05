## ECMQV 协议

DHE, Menezes, Qu and Vanstone, 1998.

预定义:

- $f$: $f=\lfloor \log_{2}q\rfloor + 1$
- $G$: $p$ 阶椭圆曲线循环群, $P$ 为其生成元.
- $Q_{A},\ d_{A}$: 公私钥对.
- $\hat{R}=(\hat{X}\pmod {2^{[f/2]}})+2^{[f/2]}$, $\hat{x}$ 指 $R$ 的横坐标二进制数据转化为整数.
- KDF: 密钥生成函数, 由哈希函数构成.
- MAC: 消息认证码

步骤:

1. A 选择随机数 $r_{A}\in[1,q-1]$, 计算 $R_{A}=r_{A}\cdot P$, 发送 $(R_{A},\ A)$
2. B 选择随机数 $r_{B}\in[1,q-1]$, 计算 $R_{B}=r_{B}\cdot P$, $s_{B}=(k_{B}+\hat{R}_{B}\cdot d_{B})$, $Z=s_{B}(R_{A}+\hat{R}_{A}\cdot Q_{A})$, $(k_{1},k_{2})\leftarrow KDF(x_{Z})$, $t_{B}=MAC_{k_{1}}(2,B,A,R_{B},R_{A})$, 发送 $R_{B},\ t_{B},\ B$
3. A 计算 $s_{A}=(k_{A}+\hat{R}_{A}\cdot d_{A})\pmod n$, $Z=s_{A}(R_{B}+\hat{R}_{B}\cdot Q_{B})$, $(k_{1},k_{2})\leftarrow KDF(x_{Z})$.
4. A 验证 $t_{B}=MAC_{k_{1}}(2,B,A,R_{B},R_{A})$, 计算 $t_{A}=MAC_{k_{1}}(3,A,B,R_{A},R_{B})$, 发送 $t_{A}$
5. B 验证 $t_{A}=MAC_{k_{1}}(3,B,A,R_{B},R_{A})$
6. A 和 B 共享密钥为 $k_{2}$

$s_{B}$ 视为对 $R_{B}$ 的隐式签名, A 计算 Z 时, 可以间接验证.

验证 $t_{a}, t_{b}$:

- 对方确实计算了 Z
- 对方知道自己在和谁通信
- 双方的通信过程没有被篡改

> [SM2](../../公钥密码/ECC/SM2.md) 密钥协商体毛级复制了这个.
