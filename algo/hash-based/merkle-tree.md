

验证某个数据局部 $D$ 是否被篡改/损坏，只需验证其 Merkle Tree 路径上的哈希值正确性，无需计算完整数据块哈希。

Merkle Tree 在 Crypto 中使用较多，为了避免哈希碰撞，建议使[密码学哈希函数](../../security/cryptography/msg-digests/msg-digest.md)。

```ascii
                 Root Hash
              H(H12 || H34)
                    │
          ┌─────────┴─────────┐
          │                   │
        H12                  H34
    H(H1 || H2)          H(H3 || H4)
          │                   │
     ┌────┴────┐         ┌────┴────┐
     │         │         │         │
    H1        H2        H3        H4
   H(D1)     H(D2)     H(D3)     H(D4)
     │         │         │         │
    D1        D2        D3        D4
  data1     data2     data3     data4
```