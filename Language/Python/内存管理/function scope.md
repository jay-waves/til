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

## 闭包

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
