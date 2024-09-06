ZKRP: zero knowledge range proof

- 基于 Dlog(discrete-log)-based inner product argument

- 基于 Lattic

- 基于 IOP, 本文技术:


[range proof 230207](../../../../paper/crypto/Range_proof_230207.pdf) obtains an efficient batch IOP-based zkRP supporting arbitrary ranges for arbitrary bases with logarithmic communication complexity and practically small proof size.

本文贡献: 实现了基于 IOP 的零知识范围证明技术, 并且是后量子安全的.

### 技术细节

使用的技术:

- IOP (interactive oracle proof): zk 框架
- Reed-Solomon code
- hash functions (BLAKE3)

技术细节:

- IPA (inner product argument):
	- univariate sum-check protocol
	- fast Reed-Solomon interactive oracle proof of proximity (FRI)