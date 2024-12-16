## Wiener Attack

也叫低加密指数攻击.

### 原理

当私钥 $d$ 过小且满足: $3d<n^{\frac{1}{4}}$, 且 $q < p < 2q$，使用 **[连分数理论](/Math/数论/连分数理论.md)** 在多项式时间内破解 $d$.

由前提条件可证明不等式:

1. $n\ =\ p*q\ >\ q^2$, 即 $q\ <\ \sqrt{ n }$
2. $0<n-\Phi(n) = p+q-1 <2q+q-1<3q<3\sqrt{ n }$

由于 $d*e\ \equiv\ 1\ (mod\ \Phi(n))$, 所以 $\exists\ k$, 满足  

$$d*e\ -\ k*\Phi(n)\ =\ 1$$

  
两边同除 ${d}*{n}$, 有  

$$\frac{e}{n}\ -\ \frac{k*\Phi(n)}{d*n}\ =\ \frac{1}{d*n}$$

即 $$\frac{e}{n}-\frac{k}{d}\ =\ \frac{1-k*(n-\Phi(n))}{d*n}$$

由不等式 (2) 知:  

$$\mid\frac{1-k*(n-\Phi(n))}{d*n}\mid\ <\ \frac{3k\sqrt{ n }}{d*n}\ =\ \frac{3k}{d \sqrt{ n }}$$

因为 $k\mid (d*e-1)$, d 取素数, 所以 $k<d<\frac{1}{3}n^{\frac{1}{4}}$, 因此  

$$\mid \frac{e}{n}-\frac{k}{d}\mid\ <\ \frac{3k}{d\sqrt{ n }}<\ \frac{1}{dN^{\frac{1}{4}}}$$

又因为 $2d\ <\ 3d\ <\ N^{\frac{1}{4}}$, 所以:  
$$\frac{1}{2d}\ >\ \frac{1}{N^{\frac{1}{4}}}$$

, 即:  
$$\mid \frac{e}{n}-\frac{k}{d}\mid\ <\ \frac{1}{2d^{2}}$$

由 [Legendre's theorem](../../../../../Math/数论/连分数理论.md) 知, 若 $\mid x\ -\ \frac{a}{b}\mid \ <\ \frac{1}{2b^2}$, 其中 x 为分数, ab 为不互质的正整数, 那么 $\frac{a}{b}$ 必然是 x 的一个收敛分数.  
所以 $\frac{k}{d}$ 是 $\frac{e}{n}$ 的一个渐进分数. 此时不断通过 $\frac{e}{n}$ 的连分数商不断向后遍历其渐进分数, 直到遍历到符合条件的 $\frac{k}{d}$, 依此 k 和 d 还可以进一步求出 N 的分解. 

Wiener 证明了该方法在上述前提条件下可以在**多项式时间**破解 RSA. 该条件只是一个**近似上界**.

<br>

### in python

```python
def rational_to_contfrac(x,y):
    '''
    将有理数转化为连分数商列表 -> [a0, ..., an]
    '''
    a = x//y
    pquotients = [a]
    while a * y != x:
        x,y = y,x-a*y
        a = x//y
        pquotients.append(a)
    return pquotients

def convergents_from_contfrac(frac):
    '''
    利用连分数商列表，计算其渐进分数
    '''
    convs = [];
    for i in range(len(frac)):
        convs.append(contfrac_to_rational(frac[0:i]))
    return convs

def contfrac_to_rational (frac):
    '''相反，将连分数列表转化为有理数分数形式，结果以元组表示分子分母
     '''
    if len(frac) == 0:
        return (0,1)
    num = frac[-1]
    denom = 1
    for _ in range(-2,-len(frac)-1,-1):
        num, denom = frac[_]*num+denom, num
    return (num,denom)

def wiener_attack(e,n):
    '''
    维纳攻击，利用连分数求解d
    '''
    frac = rational_to_contfrac(e, n)
    convergents = convergents_from_contfrac(frac)

    for (k,d) in convergents:
        #check if d is actually the key
        if k!=0 and (e*d-1)%k == 0:
            phi = (e*d-1)//k
            s = n - phi + 1
            delta = s*s - 4*n
            if(delta>=0):
                if is_square(delta) and (s+sqrt(delta))%2==0:
                    return d
    return -1

```

> 参考: [Quebrando RSA con fracciones continuas](/paper/Crypto/RSA连分数攻击.pdf)
