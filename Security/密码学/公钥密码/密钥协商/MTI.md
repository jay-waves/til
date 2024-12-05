MTI（Matsumoto-Takashima-Imai）密钥协商协议基于 DH 协议, 引入证书验证步骤, 防御中间人攻击, 但无需签名 (隐式密钥验证).

证书由可信第三方的证书颁发机构 (CA) 发布, 证明公钥确实属于所有者, 由 $(ID,\ PK,\ Sig_{CA}(ID, PK))$ 组成.

### 交换步骤

其中证书构成: $Cert=(ID,\ Y=g^{x},\ Sig_{CA}(ID, Y))$  

1. A 选择随机数 $r_{A}$, 计算 $R_{A}=g^{r_{A}}\pmod p$, 并将 $(Cert_{A}, R_{A})$ 发送给 B.
2. B 选择随机数 $r_{B}$, 计算 $R_{B}=g^{r_{B}}\pmod p$, 并将 $(Cert_{B}, R_{B})$ 发送给 A.
3. A 从 $Cert_{B}$ 获取 $Y_{B}$, 计算 $K=R_{B}^{x_{A}}\times Y_{B}^{r_{A}}$
4. B 从 $Cert_{A}$ 获取 $Y_{A}$, 计算 $K=R_{A}^{x_{B}}\times Y_{A}^{r_{B}}$

最终共享密钥值为 $K=g^{r_{A}x_{B}+r_{B}x_{A}}$
