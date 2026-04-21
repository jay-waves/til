==$\vert p-q\vert$ 须大于 $2n^\frac{1}{4}$==, 否则存在如下攻击:

**费马指数分解算法**原理是, 由等式: $$N\ =\ p*q\ =\ (\frac{p+q}{2})^{2}-(\frac{p-q}{2})^{2}$$, 若 $\vert p-q\vert$ 小, 则 $\frac{(p+q)^{2}}{4}$ 和 $N$ 的差也较小.

算法流程是, 不断循环尝试寻找关于N的两个平方数. 即, 让 $a=(\frac{p+q}{2})$ 从 $\sqrt(N)$ 开始递增, 直到 $(a^{2}-N)$ 是平方数为止. 因为求出了 $(\frac{p+q}{2})\text{ 和 }(\frac{p-q}{2})$, 由平方差公式: $N=(\frac{p+q}{2}-\frac{p-q}{2})\cdot(\frac{p+q}{2}+\frac{p-q}{2})$ 就是N的分解.

需要注意该方法只对奇合数有用, 因为偶合数分解后相加无法整除 2, 因此不能验证平方数. **算法复杂度为 $\mathbb{O}(\vert p-q\vert)$**. 

### in python

```python
import math

def fermat(n):
    a = math.ceil(math.sqrt(n))
    b2 = a * a - n
    b =round(math.sqrt(b2))
    while b * b != b2:
        a +=1
        b2 = a * a - n
        b =round(math.sqrt(b2))
    return a - b, a + b

def factorization(n):
    factors =[]
    stack =[n]
    while len(stack)>0:
        x = stack.pop()
        if x ==2:
            factors.insert(0, x)
            continue
        p, q = fermat(x) if x & 1 == 1 else (2, x //2) 
        if p ==1:
            factors.insert(0, q)
        else:
            stack.append(p)
            stack.append(q)
    return factors

if __name__ =='__main__':
	print(factorization(200))
```