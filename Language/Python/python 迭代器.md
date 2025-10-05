## 推导式与生成式

### list comprehension

直接使用表达式构造列表, 所有元素立刻被创建. 

```python
a = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10] 
squares = [x**2 for x in a] 
print(squares) 

>>> [1, 4, 9, 16, 25, 36, 49, 64, 81, 100]

# 创建等长空列表
[0 for _ in list]
```

### generator expression

将数据定义为*枚举器*, 然后仅在实际请求时才(惰性)生成, 并不直接加载入内存.  使用方法就是用 `()` 替代推导式的 `[]`

e.g.
```python
it = (len(x) for x in open(‘/tmp/my_file.txt’))

#注意 it 是一个枚举类型, 不是常规数据类型

roots = ((x, x**0.5) for x in it)

print(next(roots))

>>> (15, 3.872983346207417) 

# 上述表达式, 和元组不同.
tuple(x**2 for x in range(5))
```

**缺陷**: 枚举器是消耗品, 只能使用一次. *但, 嵌套使用非常快*. 

### enumerate

三种等价写法:

```python
zip(range(len(it)), it)

from itertools import count
zip(count(), it)

enumerate(it)
```

## 枚举器 (Generator)

普通形式: 
```python
def index_words(text):
	result = []
	if text:
		result.append(0)
	for index, letter in enumerate(text):
		if letter == ' ':
			result.append(index + 1)
	return result
```

使用 generator 形式: 函数中将 iterator 递回给 `yield`
```python
def index_words_iter(text):
	if text:
		yield 0
	for index, letter in enumerate(text):
		if letter == ' ':
			yield index + 1
```

当需要持久化保存 generator 时, 使用 `list()` 将其变为列表, 原 generator 被一次性消耗完, 固化在内存中. 

## 迭代器 (Iterator)

Python Iterator Protocol: `for x in foo`
1. `for x in foo` --> `iter(foo)` --> `foo.__iter__()`, 获取迭代器.
2. 重复调用 `next()` 来遍历迭代器, 直到其耗尽.


```python
class ReadVisits(object):
	def __init__(self, data_path):
		self.data_path = data_path
		
	def __iter__(self):
		with open(self.data_path) as f:
			for line in f:
				yield int(line)
```


### itertools

```python
from itertools import *

# 将迭代器链接:
chain(l1, l2)

cycle(l1)

it1, it2 = tee(l1, 2)

# 过滤
takewhile(__predicate__, it)

dropwhile(__predicate__, it)

islice(l1, 2, 4) # 就像 c++ span\


# 操作
product(l1, repeat=...)
```

### reduce 

```python
from functools import reduce

# reduce of sum
reduce((lambda x, y: x+y), list)

# reduct of multiply
reduce((lambda x, y: x*y), list)
```