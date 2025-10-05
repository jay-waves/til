## 控制流

异常处理:

```python
# can combine `try` with any of (`except`, `else`, `finally`)
try: 
	raise ExceptionXXX("log info")
	assert(C, "log inf") # say C as a logic condition
	...
except ErrorXXX:
	...
except AssertionError: # only catch exception from assert()
	...
except ErrorYYY: # catch some type of exceptions
	...
except Exception as e:# catch any exception inheriting from Exception, 
	# without SystemExit, KeyboardInterrupt, GneratorExit which inherit from BaseException
	...
else: # execute only if *no* exception caught
	...
finally: # always exelcute finally, not matter (normal exit, return , exception)
	...
```

上下文情景:
```python
with ... as ...:
	...
```

条件分支:

```python
if C : # say C a logic condition
	...
elif C :
	...
else:
	...
```

循环与迭代结构:

```python
while C:
	...
	break
	continue
else: # execute after normal exiting, without `break`
	...
```

```python
for xxx in iter_xxx:
	...
	yield xxx # new iterator
else:
	...
```

## 上下文

Python 使用 `with` 结构来创建一个上下文情景, 避免了重复的 `try/finally` 结构:

```python
lock = Lock()
with lock:
	print('Lock is held')

# same as:
lock.acquire()
try:
	print('Lock is held')
finally:
	lock.release()
```

标准做法是定义有 `__enter__, __exit__` 方法的工具类. 但 contextlib 的 `contextmanager` 简化了工作:

```python
# 使用 context 临时改变日志级别

@contextmanager
def debug_logging(level, name):
	logger = logging.getLogger(name)
	old_level = logger.getEffectiveLevel()
	logger.setLevel(level)
	try:
		yield logger # with here as yield value.
	finally:
		logger.setLevel(old_level)

with log_level(logging.DEBUG, 'my-log') as logger:
	logger.debug("some msg!")
	logging.debug('this will not print')
```

context 常用来自动管理资源的释放:

```python
with open('/tmp/output', 'w') as handler:
	handler.write('some data')
```

## 语法糖

syntatic syrup

### 链式比较

Python 的逻辑操作不是二元的, 可以多元:

```python
a < x < b

# 等价于
a < x and x < b
```

### 单行代码

可以在单行写简单的 if, for. 虽然这不是最佳实践.

```python
if condition: do_something()

for dog in dogs: bark()
```

`if` 还可以这样写:

```python
do_something() if condition else None
```

`for` 可以使用列表推导式, 详见 [集合推导式与生成式](集合推导式与生成式.md)

```python
ids = [ dog.id for dog in dogs ]
```

### 字符串变量展开

py3 提供了一种替代 `format()` 方法的占位方式:
```python
print('%d %s' %(i, content))
#注意'引号才能变量替换
```

常见的format方式:
```python
'{:02x}, {:02X}'.format(hex_num1, hex_num2)
```

也可以:
```python
f'{a}, {b}'
```

### Ellipsis

省略号是一种特殊语法, 用于作为占位符省略具体实现.

```python
def myfunc():
	...

# 等价于
def myfunc():
	pass
```

类型提示时, 表示数组元素数量未知

```python
def func(arr: List[..., int]):
	...
```

定义抽象基类:

```python
from abc import ABC, abstractmethod
class MyABC(ABC):
	@abstractmethod
	def myfunc(slf):
		...
```