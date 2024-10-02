标量乘的性能衡量ECC加密性能的重要指标.  
标量乘由点加组成, 点加主要瓶颈在于**模幂**和**模除 (模逆)** 运算

优化标量乘方法大致有三类:  
1. *优化标量乘流程*: 优化 $[k]P$ 计算流程, 如: 快速模幂算法, NAF窗口方法, 多基链, 加法链.
2. *转换坐标系*: 普通二元坐标系, Add 运算需要 1模逆+3模乘, Double 运算需要 1模逆+4模乘. 通过转变坐标系(射影坐标系, 雅可比坐标系等), 消除过程中的模逆运算.
3. *优化基础模运算*: 模逆运算采用: 扩展欧几里得算法, 费马小定理; 模乘运算采用: 蒙哥马利模乘算法, 快速约简公式优化.

## 1 优化标量乘 $[k]P$

### 快速模幂运算

将标量k看作二进制数处理, 复杂度约为 $\mathbb{O}(\log(k))$

```python
Point = NewType("Point", Tuple(int, int))

def ecc_mul(k: int, P: Point)->int:
	'''Elliptic Curve Point Multiplication
	, power mod method'''
	ans = (0, 0)
	base = P
	while k != 0:
		if k & 1 != 0:
			ans = self.add(base, ans) 
		base = self.double(base)
		k >>= 1
	return ans
```

等价于 蒙哥马利 法

```
R0 = P
R1 = 2 * P
if k_i == 0:
	R1 = R0 + R1
	R0 = 2 * R0
if # k_i == 1:
	R0 = R0 + R1
	R1 = 2 * R1
```

### NAF快速幂

> NAF 详见 [非邻接形式整数](../../../../Math/数论/非邻接形式整数.md)

使用 NAF 稀疏表示整数, 可以加速快速模幂的过程, 同样的方法可以应用于ECC的点数乘之中.

一般整数 k 表示中, 非零位 `1` 的数量大概是 $\frac{m}{2}$ (m 是 k 的比特长度, 即 $m\approx \log(k)$), 所以算法使用的操作数为 $\frac{m}{2}*A+m*D$ (其中A是点加操作, D是点倍乘操作)

在NAF整数k表示中, 非零位数量约等于 $\frac{m}{3}$, 所以操作数为 $\frac{m}{3}*A+m*D$, 减少了更费时的点加操作.


### 窗口方法

划定一个二进制窗口大小(一般取 $w=4$ 性能较好), 对窗口内进行预计算 $[d_{i}]P\text{, i}=0,\ 1,\ 2,\ \dots,\ 2^{w}-1$, 然后分解 $k$ 为 $k=d_{0}+2^{w}d_{1}+2^{2w}d_{2}+\dots+2^{mw}d_{m}$. 换言之, 每 w 位进行一次运算, 每次运算进行 w 次倍点运算和一次查表(预计算的)点加法. 

窗口法的优点是利用更多的倍点运算和更少的点加运算, 点加运算使用的模乘操作比倍点运算多. 该方法性能和快速幂类似.

伪代码
```
 Q ← 0
for i from m to 0 do
	Q ← point_double_repeat(Q, w)
	if di > 0 then
		Q ← point_add(Q, diP) # using pre-computed value of diP
return Q
```

> 窗口法进一步衍生出了 Sliding-Window 方法

## 2 优化模运算

### 模乘


### 模逆
费马小定理
欧几里得算法

## 3 转换坐标系
> to be continue...



## 参考

> [Elliptic curve point multiplication - Wikipedia](https://en.wikipedia.org/wiki/Elliptic_curve_point_multiplication#Point_doubling) 



