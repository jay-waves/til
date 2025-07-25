## 函数传参策略

### 为参数赋值方式

- 位置参数, position arg: 按位置和参数名进行匹配(默认行为)
- 具名参数, keyword arg: 按参数名进行具体分配, 如 `funct(args=value)`

### `*args`

作用: 接受多个(或零个)参数, 通常放在*位置参数*参数后. 如果接受多个参数, 以元组形式组织.
```python
def log(message, values):
	if not values:
		print(message)
	else:
		values_str = ‘, ‘.join(str(x) for x in values)
		print(‘%s: %s’ % (message, values_str))

def log(message, *values): # The only difference
	if not values:
		print(message)
	else:
		values_str = ‘, ‘.join(str(x) for x in values)
		print(‘%s: %s’ % (message, values_str))
```

缺点: 如果前面的参数个数不对, 可能会产生隐蔽的参数对应错误. 比如例子中message中参数其实没有提供, 但解释器会把values的第一个参数当作message

### 解包操作符 `*`

将 list,tuple 等集合对象拆散传参, 可以让代码很简洁. 本质是将集合对象的元素全部拷贝到临时元组中, 然后将元组中值传递给函数, 这个过程有开销. 

如, 输出列表:
```python
print(list[0], list[1], list[2], list[3],..., seq=',', end='') #设想list有很多元素，此时几乎不得不写一个单独for来输出
print(*list, seq=',', end='')
```

解包函数返回值：
```python
def funct():
	...
	return x, y, z
print(
	  '{:02x}, {:02x}, {:02x}'.format(*(funct()))
)
# 直接使用format(funct())是错误的，那样会将函数的tuple返回值(x,y,z)直接返回给format()
```

### 具名参数

Python 支持直接指明参数名来传参, 关键词参数应置于参数列表最后.

1. 利于阅读: 试比较: `remainder(20, 7)` 和 `remainder(number=20, divisor=7)`. 
2. 为参数提供默认值:

```python
def flow_rate(weight_diff, time_diff, period=1):
	return (weight_diff / time_diff) * period
flow_per_seconde = flow_rate(weight_diff, time_diff)
flow_per_hour = flow_rate(weight_diff, time_diff, period=3600)
```

3. 保留函数向前兼容性, 同时扩展参数

```python
def flow_rate(weight_diff, time_diff, period=1, unit=10):
	return (weight_diff / time_diff) * period * unit
	# 添加参数 unit, 而不影响之前的代码.
```

缺点是, 具名参数支持和位置参数混用, 可按顺序非具名传参给具名参数.

#### 注意

python解释器对于函数的定义只会执行一遍, 这可能导致一些貌似正确的用法产生错误.

- 函数调用只执行一次错误
```python
# error 这个方法只会在函数声明时被执行一次
def log(message, when=datetime.now()): 
	print(‘%s: %s’ % (when, message))

# 解决办法是: 在函数内部解决问题, 默认使用none作为判断条件, 并将具体情况在docstring言明
def log(message, when=none): 
	'''Log a message with a timestamp.
	Args:
		message: Message to print.
		when: datetime of when the message occurred.
			Defaults to the present time.
	'''
	when = datetime.now() if when is None else when
	print(‘%s: %s’ % (when, message))
```

- 动态类型只声明一个对象错误
```python
# error 这个方法的default默认列表只会被声明一次, 后续每次调用都使用它, 不会重新开辟
def decode(data, default={}):
	try:
		return json.loads(data)
	except ValueError:
		return default

# 正确操作: 使用none在函数内部解决, 在docstring中言明行为
def decode(data, default=None):
	'''Load JSON data from a string.
		Args:
			data: JSON data to decode.
			default: Value to return if decoding fails.
				Defaults to an empty dictionary.
	'''
	if default is None:
		default = {}
	try:
		return json.loads(data)
	except ValueError:
		return default
```

#### 强制调用者(caller)遵循具名参数方法

这可以保证你的函数调用者必须清晰明确地调用你的函数. 举例而言:

```python
def safe_division(number, divisor, ignore_overflow=False, ignore_zero_division=False):
	try:
		return number / divisor
	except OverflowError:
		if ignore_overflow:
			return 0
		else:
			raise
	except ZeroDivisionError:
		if ignore_zero_division:
			return float(‘inf’)
		else:
			raise
			
# 不规范调用者
safe_division(20, 7, True)
safe_division(20, 7, True, False)
# 正确调用者
safe_division(20, 7, ignore_overflow=True)
safe_division(20, 7, ignore_overflow=True, ignore_zero_division=True)
```

通过一点对函数声明的小小修改, 就能强制调用者对具名参数进行规范使用:

```python
def safe_division_c(number, divisor, *, ignore_overflow=False, ignore_zero_division=False):
	# 函数主体一样
```

`*` 标志着位置参数的结束, 和具名参数的开始, 此时对后面两个参数的调用必须使用具名参数的形式.

### 不同类型对象传参策略

- 不可变对象: 值传递, 即将参数值拷贝一份, 传递入的参数实际上是一个新变量. 新变量的改变不会影响外部同名该变量的值
- 可变对象: 引用传递, 即将对象的地址传入. 此时对可变对象的改变会影响到函数的外部该变量的值.

#### 区分位置参数和具名参数

- `/` 位置参数分隔符, 其之前的参数只能以位置方式传递, 不能具名传递.
- `*` 具名参数分隔符, 其之后的参数必须以关键字方式传递, 不能以位置方式传递.

## 函数返回值

### 返回多个值

1. 返回多个数值其实是通过元组实现的, 而元组可以省略括号, 所以通常看来是这样:
```python
def func(a, b):
	return a, b, c
```
2. 接受函数返回的多个数值, 用的方法有两种. 其一是利用弃用命名字符`_`, 另一种是使用\*通配符
```python
_, _, a = func(a,b)
# or
*b, a = func(a,b)
```

利用解包的性质，可以实现 `swap(a, b)` 功能：`a, b = b, a`

*实际上, 这可以理解为利用了一次元组'不可变'性质. `b, a` 其实是 `(b, a)` 缩写, RHS 先被赋值给一个元组, 再由元组进行赋值, 由于元组是不可变的, 所以 `b, a` 不需要重新开辟新地址*

## 函数作用域

python 中变量 "引用" 和 "赋值" 的方式不同, 作用域也不同.

对于变量引用, 子作用域可以任意引用其父域的变量. 

```python
def sort_priority(values, group):
	def helper(x):
		if x in group: #group 是定义helper的父域中的变量，但是可以引用使用
			return (0, x)
		return (1, x)
	values.sort(key=helper)
```

事实上, 当变量引用时, 解释器会**依次**遍历以下作用域去解释这个变量:
1. 当前函数作用域
2. 闭包作用域, 即包含当前局部作用域的外层作用域
3. 全局作用域 global, 或者称为 module
4. 内建变量, built-in scope  
5. 如果各处都找不到被引用的变量, 产生 `NameError` 异常

对于变量赋值 (值改变), 子作用域不能修改父域的不可变变量, 否则会被解释为新声明变量. **但是可以直接修改全局可变变量.**

```python
def sort_priority2(numbers, group):
	found = False
	def helper(x):
		if x in group:
			found = True # mistake!! 
			return (0, x)
		return (1, x)
	numbers.sort(key=helper)
	return found
```

当函数局部试图修改父作用域的不可变变量, Python 会默认创建新变量. 使用 `nonlocal` 关键字来声明: 确实要修改父作用域的该名称不可变变量.

```python
def sort_priority3(numbers, group):
	found = False
	def helper(x):
		nonlocal found
		if x in group:
			found = True
			return (0, x)
		return (1, x)
	numbers.sort(key=helper)
	return found
```

当函数局部试图修改全局不可变变量时, Python 默认会重新创建一个新变量. 使用 `global` 关键字来声明: 确实要修改全局的该名称变量.

```
...
```

当然, 修改变量作用域是比较危险的行为. 更好的方法是定义一个类:

```python
class Sorter(object):
	def __init__(self, group):
		self.group = group
		self.found = False
	def __call__(self, x):
		if x in self.group:
			self.found = True
			return (0, x)
		return (1, x)

sorter = Sorter(group)
numbers.sort(key=sorter)
assert sorter.found is True
```

### 闭包

当你在一个函数中定义另一个函数, 并在内部函数中引用外部函数的局部变量, 这个内部函数就可以作为闭包来 " 捕获 " 并保留对这些局部变量的访问权. 但是, 如果你只是在一个函数 (作为闭包) 中调用另一个外部的函数, 那么这个外部函数默认情况下是不能直接访问闭包的外部变量的, 除非你明确地将这些变量作为参数传递给它. 

考虑以下例子:

```python
def outer(x):     
	def inner():         
		return x  
		# inner 函数可以直接访问外部函数的局部变量 x     
	return inner  
closure = outer(10) 
print(closure())  # 输出：10  

def another_function(y):     
	return y  
	
def outer_2(x):     
	return another_function(x)  
	# 虽然在outer_2中调用，但another_function不能直接访问x，除非作为参数传递  
print(outer_2(20))  # 输出：20
```

在第一个例子中, `inner` 是一个闭包, 它可以直接访问外部函数 `outer` 的局部变量 `x`.

在第二个例子中, `outer_2` 调用 `another_function` 并传递 `x` 给它. 尽管 `another_function` 在 `outer_2` 的上下文中被调用, 但它不能直接访问 `outer_2` 的局部变量 `x`, 除非 `x` 被明确地作为参数传递给它.

### 使用可调用对象而不是闭包来实现有状态函数

对于需要状态的函数, 可以用闭包 (类似 C 的局部静态变量), 也可以定义一个类, 并且类实现 `__call__` 调用方法来直观调用.

```python
calss CountMissing(object):
	def __init__(self):
		self.added = 0
	def __call__(self):
		self.added += 1
		return 0

counter = CountMissing
assert callable(counter)
defaultdict(counter)
```

闭包的方法并不直观:

```python
def ...(...):
	added_count = 0

	def missing():
		nonlocal added_count # stateful closure
		added_count += 1
		return 0
		
	defaultdict(missing)
```
