## Feistel 结构

![|400](../../../../attach/密码学_Feistel结构.avif)

对于每次迭代:  
$L_{i+1}=R_{i}$  
$R_{i+1}=F\ (R_{i},\ K_{i})\oplus L_{i}$

解密结构与加密相同, 但需调换 $L,\ R$ 位置:  
$R_{i}=L_{i+1}$  
$L_{i}=F\ (L_{i+1},\ K_{i})\oplus R_{i+1}=F\ (R_{i},\ K_{i})\oplus F\ (R_{i},\ K_{i})\oplus L_{i}$

本质是通过还原 $k=F\ (R_{i},\ K_{i})$, 来还原明文 $k\oplus k\oplus L_{i}=L_{i}$. 在DES实现中, 先调换 $L,\ R$ 顺序, 再进行 $IP^{-1}$ 变换, 同时解密的轮密钥以相反顺序注入.
