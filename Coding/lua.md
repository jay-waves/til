Lua 是一款方便嵌入 C/C++ 代码之中的轻量级脚本语言. 

```bash
lua hello.lua
```

导入其他 lua 脚本:

```lua
dofile('lib1.lua')
```

注释语法:

```lua
-- one line comments

--[[ 
	multi-line long comment
--]]

--[[ 
	print(10)  --> 10, commented
--]]

---[[ 
	print(10)  --> 10, uncommented
--]]
```

栈大小为一百万个实体.

## 类型系统

Lua 是动态类型的语言, 未初始化的变量值一律为 `nil`. `nil` 的语义就是空值, 赋值为 `nil` 的任意变量都将从内存被删除 (GC).

Lua 基础类型有:

```lua
type(nil)          --> nil
type(ture)         --> boolean
type(10.4)         --> number
type("Hello")      --> string
type(io.stdin)     --> userdata
type(type)         --> function
type({})           --> table
type(type(...))    --> string

type(a)            --> nil
a = 10
type(a)            --> number
a = "hello"   
type(a)            --> string
```

`userdata` 可用于存储任意 C 程序的数据, 如任意用户定义的结构体. 变量本身没有类型, 跟随其蕴含的值的类型.

未初始化变量的默认值都是 `nil`. 对于已存在变量 `x`, 使用 `x=nil` 将其内存删除. (内置 GC)
```lua
foo = anUnkownVariable  --> foo = nil
```

### 布尔类型

Lua 提供 `and, or, not` 关键字, 也支持表达式 `<, >, <=, >=, ==, ~=`

| type    | condition test |     |
| ------- | --------- | --- |
| `false` | false     |     |
| `nil`   | false     |     |
| `0`     | true      |     |
| `""`    | true          | empty string    |

`and` 和 `or` 都是短路判断 (short-circuited). 可以用来模拟 C 的三目运算符 `a?b:c`

```lua
aBoolValue = false
ans = aBoolValue and 'yes' or 'no'  --> 'no'
```

**只有 `false` 和 `nil` 会判定为否, 0 和空字符串 `''` 都判定为真.**

### 数字类型

Lua 只有两种数字类型: 64 位整数, 和双精度浮点数. 当然 Lua 也可以被编译在 32 位平台上 (称为 Small Lua), 使用 32 位整数和单精度浮点数. 整数直到 Lua5.3 才被正式引入, 之前都用双精度浮点数.

```lua
0.4
4.57e-3
0.3E+20
0x1A3

type(3.0)      --> number
type(3)        --> number

math.type(3.0) --> float 
math.type(3)   --> integer
```

常见数字运算符: 对于除法和指数操作, Lua 强制将结果转化为浮点数.

```lua
- + * /  ^
//   --> floor division
%    --> a % b == a - ((a // b) * b)
```

取余操作常用于获得固定位数的浮点数:

```lua
x = math.pi
x - x % 0.01   --> 3.14
x - x % 0.001  --> 3.141
```

数字类型转化: 由于双精度浮点数的最高实际有效位位数只有 53 位, 所以有效位数更多的整数转化为浮点数时会有精度损失. 这可能会导致 Lua5.3 和之前版本程序的不兼容性.

```lua
3 + 0.0  --> 将整数转化为浮点数
2^53 | 0 --> 将浮点数转化为整数. 注意被转化的浮点数只能是 0~2^53 的整数值, 否则会报错
math.tointeger(2^64) --> nil, 无法不丢失精度地转化时, 不显式报错, 返回 nil
```

常用数学库函数:

```lua
math.huge         --> inf
math.sin(math.pi / 2)
math.max(10.4, 7, -3, 20)

math.random()     --> uniform distribution in [0,1)
math.random(n)    --> integer in [1,n]
math.random(n, m) --> integer in [n,m]
--[[ 
对于随机数函数, Lua 解释器默认使用的种子总为 `1`, 
这意味着每次独立运行程序时, 产生的随机数序列是相同的.
]]
math.randomseed(os.time())

math.floor(3.3)      --> 3
math.floor(-3.3)     --> -4
math.ceil(3.3)       --> 4
math.ceil(-3.3)      --> -3
math.modf(3.3)       --> 3   0.3
math.modf(-3.3)      --> -3  -0.3

--[[ 
浮点数形式表示的整数取整时, 可能因精度误差导致错误.
]]
function round(x)
	local f = math.floor(x)
	if (x == f) or (x % 2.0 == 0.5) then 
		return f --> for 2.5 or 2, return 2
	else
		return math.floor(x + 0.5) --> nearest integer to x
	end
end
```

### 字符串

在 Lua 中, 字符串是不可变类型, 由任意大小字节串表示. 字符串可用单引号, 也可用双引号, 示字符串内容灵活选择. Lua5.3 引入了更广泛的 UTF-8 支持.

```lua
a = "hello"
b = "hello\nworld"
c = 'hello\nworld'
d = '\049\x41\u{3b1}' --> \ddd \xhh 
e = '\u{3b1}'         --> \u{hhh..} 表示 UTF-8 字符, Lua5.3 引入
f = [[ 
	<html>
	<head>
		..... 长字符串
	</head>
	...
]]
g = [===[
	如果字符串中包含 [[]], 可以在中括号之间添加任意数量的等号
]===]

```

常用字符串库函数: 由于字符串是不可变类型, 所以库函数通常返回一份修改后的副本 (一个新字符串).

```lua
a = "hello"
#a                    --> 获取字节长度, 取决于具体的字符串编码方式.
string.len(a)
"hello " .. "world"   --> 链接操作.

tonumber("100101", 2) --> 37
tostring(10)          --> "10"

string.rep("a", 3)    --> "aaa"
string.reverse("abc") --> "cba"
string.lower(s)
string.lower(s)
string.sub(s, i, j)   --> 提取字符串的一部分 s[i,j]

string.char(97)       --> "a"
string.byte("a")      --> 97

string.format("%s %d %x", 10, 200, "a") --> "10 c8 a"
string.format("%.4f %02d", math.pi, 1)  --> "3.1416 01"

-- 库函数也可以作为对象的"方法"被使用
s:upper()

-- UTF-8 标准库, 在字面值语义上, 而不是在 Unicode 层面操作字符串.
utf8.len("你好")           --> 2
utf8.len("ab\x93")         --> nil  3  非法的 UTF-8 字节串, 将返回 nil 和出错位置
utf8.char(133, 233, ...)
utf8.codepoint("你好", 2)  --> 类似 stirng.byte(), 但返回值无语义
utf8.offset("你好", 2)     --> 获取 "好" 在实际字节种的偏移
utf8.codes("你好")
	--> 1   65
	--> 2   231

```

### 字典

lua 字典使用哈希表实现.

```lua
t = {k1 = 'v1', k2 = false}
print(t.k1) --> 'v1'
t.k3 = {}
t.k2 = nil  --> remove k2

-- 实际上任意类型 (除了 nil) 都可以作为键
{['@!xxx'] = 'xxx', [{}] = 1111, [6.28] = 'xxx'}  --> [{}] 实际是 nil

-- 迭代
for k, v in pairs(t) do 
	...
end 
```

lua 没有提供数组类型, 也鼓励用 *字典* 实现. 

```lua
arr = {"apple", "banana", "cherry"} --> 自动编号, 但从 1 开始!!!

print(arr[1]) --> 'apple'
#arr --> 数组长度
```

## 控制流


```lua
while num < 50 do
	num = num + 1  --> 没有 ++ 和 += 操作
then 

if num > 40 then 
	print('over 40')
elseif s ~= 'walternate' then 
	io.write('not over 40\n')
else 
	local line = io.read()  --> 读取 stdin 行
	print('xxx' .. line)    --> 用 .. 连接字符串
end 

repeat 
	print('xxx')
	num = num - 1
until num == 0

for i = 1, 100 do   --> range: begin, end[, step]
	print(i)
end	

```

### 函数

```lua
function fib(n)
	if n < 2 then return 1 end 
	return fib(n-2) + fib(n-1)
end

-- closures & anonymous functions. 函数是第一类公民
function adder(x)
	return function(y) return x + y end 
end 

a1 = adder(9)
a1(16) --> 25

-- 传递参数列表: 少的赋为 nil, 多余的丢弃.
x, y, z = 1, 2, 3, 4  --> 4 被扔掉
```

## 元编程

lua 提供了一种重载 *字典* 类型的操作符的方式.

```lua
f1 = {a = 1, b = 2} -- Represents the fraction a/b. 
f2 = {a = 2, b = 3} 

-- This would fail: 
-- s = f1 + f2 

metafraction = {} 
function metafraction.__add(f1, f2) 
	sum = {}
	sum.b = f1.b * f2.b 
	sum.a = f1.a * f2.b + f2.a * f1.b 
	return sum 
end 

setmetatable(f1, metafraction) 
setmetatable(f2, metafraction) 

s = f1 + f2 -- call __add(f1, f2) on f1's metatable
```

可重载的方法称为 `metamethods`, 包括:
- `__add(a, b)` --> `a + b`
- `__sub(a, b)` --> `a - b`
- `__mul(a, b)` --> `a * b`
- `__div(a, b)` --> `a / b`
- `__mod(a, b)` --> `a % b`
- `__pow(a, b)` --> `a ^ b`
- `__unm(a)`    --> `-a`
- `__concat(a, b)` --> `a .. b`
- `__len(a)`    --> `#a`
- `__eq(a, b)`  --> `a == b`
- `__lt(a, b)`  --> `a < b`
- `__le(a, b)`  --> `a <= b`
- `__index(a, b)`  --> `a.b`, fn or table 
- `__newindex(a, b, c)` --> `a.b = c`
- `__call(a, ...)` --> `a(...)`

### 类

不内置类, 需要借助 *字典* 类型的方法重载来实现. `table:fn(...)` 实际等价于 `table.fn(self, ...)`, 是一种语法糖, 此时第一个参数被默认命名为 self.

```lua
Dog = {}

function Dog:new()
	newObj = {sound = 'woof'}
	self.__index = self 
	return setmetatable(newObj, self)
end

function Dog:makeSound()
	print('I say' .. self.sound)
end 

mrDog = Dog:new()
mrDog:makeSound()  --> 'I say woof'
```

```lua
myDog = {
	name = "John",
	year = 13,
	woof = function(self)
		print(self.name .. ' say woof')
	end,
}

myDog.woof(myDog)
myDog:woof()
```

#### 继承

```lua
LoudDog = Dog:new()

function LoudDog:makeSound() -- 直接重载字典值即可
	s = self.sound .. ' '
	print(s .. s .. s)
end
```

## 模块化

`mod.lua`  包含如下
```lua
local M = {}

local function syMyName()
	print('xxx)
end 

function M.sayHello()
	print('Why hello there')
	sayMyName()
end 

return M
```

另一个文件这样使用 `mod.lua` 模块:
```lua 
local mod = require('mod') -- run file 'mod.lua'

--[[ require 实际将 mod.lua 脚本包装为函数体. 因为 mod.lua 返回了 M, 所以 mod = M
	local mod = (function ()
		... contents of mod.lua
	end)
--]]

mod.sayHello() 

mod.sayMyName()  --> fail. sayMyName 不可见
```

## 调用 C



## 参考

Programming in Lua. Fourth edition.

https://tylerneylon.com/a/learn-lua/