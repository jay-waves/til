## Algebra

### Linear Algebra

矩阵:
```wolfram
{
	{1, 2}
	{3, 4}
}
```

```wolfram
Det[..] (* 计算矩阵的行列式 *)
Inverse[..] (* 计算矩阵的逆 *)
Eigenvalues[]
Eigenvactors[]

LinearSolve[] (* 线性方程组 *)
```

### Polynomial

解方程组, 支持线性与非线性系统
```wolfram
Solve[]
Reduce[]
Eliminate[] 
Resolve[]
SolveValues
```

多项式约简:
```wolfram
PolynomialReduce

```

多项式展开与因式分解:
```wolfram
Expand
Factor
Collect
```

符号化:
```wolfram
FullForm[expr]
TreeForm[expr]
TraditionalForm[expr]
```
![|300](../../../attach/Pasted%20image%2020240502130906.png)

### Finite Field

```wolfram
FiniteField
FiniteFieldElement
Modulus[]
PoynomialMod[]

```

## Calculus

求导数:

```wolfram
D[f[x], x]
```

求积分:

```wolfram
Integrate[x^2, x]
```

求极限:

```wolfram
Limit[sin(x)/x, x->0]
```

### Series

幂级数展开:

```wolfram
Series[Exp[x], {x, 0, 5}]
```

级数求和, 也可用于求和函数:

```wolfram
Sum[1/^n^2, {n, 1, 10}]
Sum[x^n, {n, 0, Infinity}]
```

## Probabilistic

正态分布:

```wolfram
Normalistribution[0, 1]
```

计算正态分布中 $x>1$ 的概率:
```wolfram
Probability[x>1, x \[Distributed]Normalistribution[0, 1]]
```

计算期望:
```wolfram
Expectation[x^2, x, ...]
```

随机数:
```wolfram
RandomVariate[NormalDistribution[0, 1], 10] (* 生成 10 个随机数, 服从标准正态分布*)
```

## Number

```wolfram
Prime[] (* the n^th prime *)
PrimeQ[] (* test for primality *)
NextPrime[]
FactorInteger[100]

GCD[]
ExtendedGCD[]
LCM[]

ChineseRemainder[]
Mod[]
PowerMod[]
PrimitiveRoot[]

Binomial[n, k] (* C(n, k) *)
n! / (n - k)!  (* A(n, k) *)
Pochhammer[x, n]
```

Pochhammer 符号, 也称为上升阶乘, 定义为: $$(x)_{n}=x(x+1)(x+2)\dots(x+n-1)=\frac{\Gamma(x+n)}{\Gamma(x)}$$

$(x)_{0}=1$, 

排列数可以用 Pochhammer 符号符号表示: $$A(n,k)=(n-k+1)_{k}$$