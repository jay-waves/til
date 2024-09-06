**CCA, Choice CipherText Attack, 选择密文攻击**是针对 **Plain-RSA, 教科书式 RSA** 的有效攻击手段, 但经过 [OAEP 填充](../OAEP-填充.md)等引入随机的填充方式后, 同一明文对应不同.

### 攻击流程

假设敌手有选择密文攻击能力, 密文 $C$ 和明文 $M$, 选取随机数 $r$. 

此时敌手已知 $(C, e, N, r)$

1. 让A解密 $r^eC$. 因为 $r^eC=r^eM^e$, 所以解密结果为 $rM$
2. 计算 $r^{-1}\pmod{n}$
3. 计算 $M=r^{-1}\times rM\pmod{n}$

> 类似攻击还有: [RSA-篡改攻击](RSA-篡改攻击.md)