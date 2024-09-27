---
url: https://docs.python.org/3/library/math.html
license: PDL
---

`math` 属于 Python 标准库, 提供 C 标准的数学函数. 如果需要支持复数, 请使用标准库中的 `cmath` 的同名函数.

### Number-Theoretic 

Ceil: $\lceil x\rceil$

```python
math.ceil(x)
```

Floor: $\lfloor x\rfloor$

```python
math.floor(x)
```

Binomial Coefficient: $$C_{n}^{k}=\frac{n!}{k! \cdot (n-k)!}$$

```python
math.comb(n,k)
# return 0 if k > n
# raise TypeError if arguments are not integers
# raise ValueError if arguments are negative
```

$$A_{n}^{k}=\frac{n!}{(n-k)!}$$

```python
math.perm(n, k=None)
# k defaults to n, then returns n!
```

Abosolute Value: $|x|$

```python
math.fabs(x)
```

Modulus of x on y: $x\pmod y$

```python
math.fmod(x, y)
```

Factorial of N: 

```python
math.factorial(n)
# raise ValueError if n is not integral or is negative
```

$x=m\times 2^{e}$, where $0.5\leq |m|<1$ .

```python
(m,e) = math.frexp(x) 
```

$m\times 2^{2}$

```python
math.ldexp(m, e) 
# essentially the inverse of frexp()
```

The greatest common divisor:

```python
math.gcd(*integers)
math.lcm(*integers)
```

Test approximate equality:

```python
math.isclose(a, b, *, rel_tol=1e-09, abs_tol=0.0)
# return true if a-->b
# IEEE754 NaN is not close to any, 
# IEEE754 inf/-inf is only close to themselves.
```

$\lfloor\sqrt{x}\rfloor$

```python
math.isqrt(x)
# equivalently the greatest integer a such that a^2<=n
```

```python
(a: float, b: float) = math.modf(x)
# a: fractional parts with sign of x
# b: integer parts with sign of x
```

$$\prod x_{i}$$

```python
math.prod(iterable, *, start=1)
```

math.copysign

math.fmod

math.fsum

### Power and Logarithmic

Cube Root: $\sqrt[3]{x}$

```python
math.cbrt(x)
```

Squqre Root: $\sqrt{x}$

```python
math.sqrt(x)
```

...


$x^{y}$

```python
math.pow(x, y)
# raise ValueError if y is not integal and x is negative.

# math.pow() converts x,y to float. instead, use ** or 
# built-in pow() to compute exact integer powers
```

> $(-4)^{0.9}$ 需要引入复数, 不在标准 math 库中, 参见 cmath 库.

### Trigonometric

Arc Cosine of x in radians: $\arccos (x)$

```python
math.acos(x) 
```

Arc tangent of x, in radians: $\arctan(x)$

```python
math.atan(x)
```

Euclidean distance between two points:

$$|p-q|=\sqrt{\sum^{n}_{i=1}(p_{i}-q_{i})^{2}}$$

```python
math.dist(p: iterable, q: iterable)
```

```
math....
```

```python
math.degrees(x)
# convert angle x from radians to degrees
math.radians(x)
```

### Hyperbolic

Arc Hyperbolic Cosine: $$arccosh(x)=ln(x+\sqrt{x^{2}-1})$$

```python
math.acosh(x)
```

Arc hyperbolic Sine: $$arcsinh(x)=ln(x+\sqrt{x^{2}+1})$$

```python
math.asinh(x)
```

Arc Hyperbolic Tangent: $$arctanh(x)=\frac{1}{2}ln\left( \frac{1+x}{1-x} \right)$$

```python
math.tanh(x)
```

Hyperbolic Cosine: $$\cosh(x)=\frac{e^{x}+e^{-x}}{2}$$

```python
math.cosh(x)
```

Hyperbolic Sine: $$\sinh(x)=\frac{e^{x}-e^{-x}}{2}$$

```python
math.sinh(x)
```

Hyperbolic Tangent: $$\tanh(x)=\frac{\sinh(x)}{\cosh(x)}$$

```python
math.tanh(x)
```

### Special

```python
math.erf(x)
```

...

```python
math.gamma(x)
```


### Constants

$\tau=2\cdot \pi$

```python
math.pi

math.tau # 2pi

```

$e$

```python
math.e
```

```python
math.inf # = float('inf') = IEEE754 Inf

math.nan # IEEE754 NaN = float('nan')
```

```python
math.isfinite(x)
# True if x is neither an infinity nor a NaN

math.isinf(x)
# True if x is IEEE754 -inf or inf

math.isnan(x) # instead of nan == nan
```