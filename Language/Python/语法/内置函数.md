---
source: https://docs.python.org/zh-cn/3.12/library/functions.html
---

> 没有收录所有异步函数 async_, 没有收录内存管理, 没有收录 Python 解释器相关

## 内置类型转换


### float

返回一个浮点数, 参数可以是数值或字符串. 字符串可以包含:

```
+ - inf Infinity INF nan NAN NaN (大小写无影响)
123321 123_321 123.321 123e-321 +123E321 
```

浮点数超范围时, 触发 OverflowError.

### int

返回一个整数, 无参数时返回 `0`

### bool

### tuple

`tuple` 是不可变的序列类型.

### dict

返回一个字典类型.

### list

返回可变序列类型 list.

### str

```python
class str(object=b'', encoding='utf-8', errors='strict')
```

### slice

返回一个切片对象.
```python
class slice(start, stop, step=None)
```

注意 `start, stop, step` 都是值函数, 如 `a[start:stop:step]`

### set

`class set(iterable)` 返回新的 set 对象.

### frozenset

`class frozenset(iterable=set())` 返回内置类型 frozenset.

### bytearray

### bytes

### chr

`chr(i)` 返回 Unicode 码位为整数 `i` 的字符串格式, 如 `chr(97)` 返回字符串 `a`.

是 `ord()` 的逆函数.

### complex

将字符串或数字转化为一个复数对象.

```python
complex('+1.23')
complex('-4.5j')
complex('\t( -1.23+4.5J)\n')
complex('-Infinity+NaNj')
complex(real=-1.23, imag=4.5)
```

***

## 元编程

### id

在对象生命周期内, 标识值 `id` 是恒定且唯一的. Cpython 中特指对象的内存地址.

### hash

返回对象的哈希值. 哈希值是整数, 在字典查找元素时, 用于快速比较字典的键. 相同数值有形同的哈希, 如 `1` 和 `1.0`

### type

返回 `object` 的类型, 与 `object.__class__` 相同. 推荐配合 `isinstance()` 使用.

```python
class type(object)
```

### isinstance

### issubclass

### vars

返回任何对象的 `__dict__` 属性.

```python
vars(object)
```

### callable

检查对象是否是可调用的. 注意, 返回 `True`, 调用也可能失败; 返回 `False`, 调用则一定会失败. 实例有 `__call__()` 式, 就可以被调用.

### setattr

`setattr(object, name, value)` 为对象设置一个新属性 (一个键值对)

### hasattr

`hasattr(object, name) -> bool` 通过调用 `getattr` 来查看是否有 AttributeError.

### getattr

`getattr(object, name, default)` 查找 object 中指定属性的值. 如果没有 default 且没有找到, 返回 AttributeError.

### repr 

`repr(object)` 返回一个对象的可打印形式字符串. 可通过定义 `__repr__()` 来控制输出.

### dir

`dir()` 返回本地作用域的名称列表

`dir(object)` 返回对象的有效属性列表. 如果有, 会调用 `__dir__()` 方法.

```python
import struct
>>> dir()   # show the names in the module namespace  
['__builtins__', '__name__', 'struct']
>>> dir(struct)   # show the names in the struct module 
['Struct', '__all__', '__builtins__', '__cached__', '__doc__', '__file__',
 '__initializing__', '__loader__', '__name__', '__package__',
 '_clearcache', 'calcsize', 'error', 'pack', 'pack_into',
 'unpack', 'unpack_from']
```

### globals

返回当前模块命名空间的字典.

### iter

返回一个 iterator 对象
- `iter(object)`: 对象必须有 `__iter__()` 方法或支持序列协议, 否则触发 TypeError
- `iter(object, sentinel)`: 对象必须可调用, 此时创建的迭代器将在被调用 `__next__()` 方式时, 不带参数的调用 `object`. 如果返回值等于 `sentinel`, 触发 StopIteration

第二种方式可用于构建块读取器. 如从二进制文件以固定长度读取块, 直到末尾.
```python
with open('....db', 'rb') as f:
	for block in iter(partial(f.read, 64), b''):
		process_block(block)
```

***

## 面向对象

### super

返回对象的父类, 用于链式调用父类或兄弟类. 继承关系的继承关系遵循 `__mro__` 属性.

### property

```python
class property(fget=None, fset=None, fdel=None, doc=None)

# fget 获取属性值的函数
# fset 设置属性值的函数
# fdel 删除属性值的函数
```

定义托管属性 `x`: 
- `c.x` 将调用 getter, 
- `c.x = ...` 将调用 setter, 
- `del c.x` 将调用 deleter.

```python
class C:
    def __init__(self):
        self._x = None

    def getx(self):
        return self._x

    def setx(self, value):
        self._x = value

    def delx(self):
        del self._x

    x = property(getx, setx, delx, "I'm the 'x' property.")
```

也可以用装饰器实现类似效果:

```python
class C:
	def __init__(self):
		self._x = none
	
	@property
	def x(self)
		return self._x

	@x.setter
	def x(self, value):
		self._x = value

	@x.deleter
	def x(self):
		del self._x
```

***

## 可迭代操作

注意 `iterable` 是可迭代对象, 可以是序列, 支持迭代的容器, 迭代器.

### range

`range(start, stop, step=1)` 返回一个不可变序列类型.

### all 

当所有元素都为真, 或者 `iterable` 为空, 则返回 `True`

```python
all(iterable)
```

### any

`iterable` 的任一元素为真值, 则返回 `True`. 如果迭代对象为空, 返回 `False`

```python
any(iterable)
```

### min

返回迭代器中最小元素, 或几个实参中最小的.

```python
min(iterator, *, key=keyfunc)
min(arg1, arg2, *args, key=keyfunc)
```

### max

类似 `min()`

### sorted

返回一个新的已稳定排序列表.

```python
sorted(iterable, *, key=None, reverse=False)

key = str.lower
key = functools.cmp_to_key(....)
key = object.__lt__()
```

### sum 

```python
sum(iterable, start=0)
```

对于字符串, 更合适的方式是 `''.json(sequence)`. 并且数值解并不精确.

### zip

将多个迭代器的第 i 项组成第 i 个元组. 也可认为 `zip()` 将行变成了列. 

```python
for my_tuple in zip([1,2,3], ['fee', 'fi', 'fo'])
```

默认情况中, 最短迭代器迭代完成后, `zip()` 即终止.

### enumerate

返回元组, 第一个元素为计数器 (从 `start` 开始), 第二个是迭代器对应元素.
```python
enumerate(iterable, start=0)
```

### map

`map(func, iterable, *iterables)` 将 `func` 应用到 `iterable` 每一项.

### filter

`filter(function, iterable)` 用应用布尔函数 `func` 后返回真值的元素构造一个迭代器.

```python
filter(func, itera) <--> (item for item in itera if func(item))

filter(None, itera) <--> (item for item in itera if item)
```

### next

`next(iterator, default)` 调用 `__next__()` 方法, 如果迭代器耗尽, 返回 `default`, 否则触发 `StopIteration`

### reversed

`reversed(seq)`, seq 必须有 `__reversed__()` 方法, 或支持序列协议 (具有 `__len__()` 方法, 并从 `0` 开始的整数参数的 `__getitem__()` 方法)

### len

`len(s)` 返回序列长度.

## 格式化

### bin

将整数转化为带前缀 `0b` 的二进制字符串.

### oct

将整数转化为带前缀 `0o` 的八进制字符串.

### hex 

将整数转化为带前缀 `0x` 的十六进制字符串.

### ascii

类似 `__repr__()`, 返回对象的可打印形式. 使用 `\x, \u` 等对 ASCII 编码的字符进行转义.

### print

`print(*objects, sep=' ', end='\n', file=None, flush=False)`

是否输出缓冲由 `file` 对象决定, 但是也可以用 `flush` 参数强制指定

### format

`format(value, format_spec='')` 格式化数值

***

## 操作系统服务

### breakpoint

`breakpoint(*args, **kws)` 在该位置进入调试器, 并传入参数.

### input

`input(prompt)` 将 `prompt` 写入标准输出, 无换行, 然后从输入中读取一行, 转化为 `str`.

读取至 EOF 时, 触发 EOFError.

### open

```python
open(file, mode='r', buffering=-1, 
		encoding=None, errors=None, 
		newline=None, closefd=True, opener=None) -> file_object
```

无法打开文件时触发 OSError. `file` 可以是路径, 或者整数型文件描述符 (有细节). 

打开方式如下:

| 字符 | 含义                                            | file_object 类型 |
| ---- | ----------------------------------------------- | ---------------- |
| `r`  | 读取 (默认)                                     |               |
| `w`  | 写入, 并先截断文件                              |                 |
| `x`  | 排他性创建, 如果已创建则失败                    |                  |
| `a`  | 写入, 用于在文件末尾追加                        |                  |
| `b`  | 二进制模式, 返回 `bytes`, 不进行编码            | io.BufferedIOBase |
| `t`  | 文本模式 (默认), 返回 `str`, 用 `encoding` 编码 | io.TextIOBase                 |
| `+`  | 用于更新????                                    |                  |

`r` 与 `rt` 同义, `w` 与 `wt` 同义. `r+`, `r+b` 打开文件但不清空内容; `w+` 和 `w+b` 会打开文件并清空内容.

`buffering` 用于设置缓冲策略:
- 文本模式下:
	- `0` 代表关闭缓冲
	- `1` 行缓冲, 默认行为.
	- `>1` 整数表示固定大小的块缓冲区的字节大小.
- 二进制文件以固定大小的块进行缓冲. 启发式设置为底层设备的块大小.

`newline` 决定如何解析来自流的换行符, 可以为:
- `None`: 通用模式, 可以为 `\n, \r, \r\n`. 读取时不做处理, 写入时转化为对应系统的默认分隔符 `os.linesep`
- `''`: 也是通用模式. 读取时同一转化为 `\n`, 写入时仍保持 `\n`
- `\r`, `\n`, `\r\n` 指定

> 新创建的文件不可继承.

### eval

执行任意序列化的 Python 字符串表达式. 该函数不安全.

`eval(expres, globals=None, locals=None)`
- `expres (str | code object)`: 一个 Python 表达式
- `globals (dict | None)`: 全局命名空间
- `locals (mapping | None)`: 局部命名空间

### exec

`exec(object, globals=None, localNone, *, closure=None)`

如果省略了可选部分, 代码将在当前作用域执行.

***

## 数学

### abs

返回数字绝对值, 可以是任何实现了 `__abs__()` 的对象.

### round

`round(number, ndigits=None)` 

### pow

`pow(base, exp, mod=None)`

### divmod

`divmod(a, b) == (a//b, a%b)`, 最好只使用整数.